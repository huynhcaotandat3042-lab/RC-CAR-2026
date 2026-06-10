/*
  ============================================================
  rx_time_milis.ino
  B5 + Indicator + FORCE INVALID = 150
  ------------------------------------------------------------
  Chuc nang:
  - Nhan packet radio tu TX qua ESP-NOW
  - Nhan packet sensor tu ESP32-SENSOR qua UART
  - Ho tro mode MANUAL / AUTO
  - AUTO dung state machine tranh vat can
  - B5:
      + Giữ WALL_NEAR_CM
      + Bỏ WALL_RELEASE_CM
      + Dùng thời gian turn-in để suy ra turn-out
  - Xi nhan:
      + Trong MANUAL, danh lai trai/phai -> den chop tat ben tuong ung
  - INVALID = 150 cm:
      + Neu sensor invalid thi ep thanh 150.0 cm va valid=true
  ============================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

// ============================================================
// 0) SAFETY / TEST MODE
// ============================================================

const bool ENABLE_PWM_OUTPUT = true;

// ============================================================
// 1) RX RADIO / ESPNOW CONFIG
// ============================================================

static const uint8_t ESPNOW_CHANNEL = 6;

// ============================================================
// 2) SENSOR UART CONFIG
// ============================================================

const int SENSOR_UART_RX_PIN = 25;
const int SENSOR_UART_BAUD   = 115200;

HardwareSerial SensorLink(2);

// ============================================================
// 3) ACTUATOR OUTPUT PINS
// ============================================================

const int STEER_SERVO_PIN = 26;
const int ESC_PWM_PIN     = 18;

// Neu test that thay servo danh nguoc co khi, doi true/false
const bool REVERSE_STEER_SERVO = true;

Servo steerServo;
Servo escServo;

// ============================================================
// 3.1) INDICATOR LIGHT PINS
// ============================================================

const int LEFT_INDICATOR_PIN  = 4;
const int RIGHT_INDICATOR_PIN = 5;

// true = HIGH thi den sang
// false = LOW thi den sang
const bool INDICATOR_ACTIVE_HIGH = true;

// Neu lenh lai lech khoi center qua nguong nay thi bat xi nhan
const int INDICATOR_STEER_THRESHOLD_US = 80;

// Chu ky chop tat
const unsigned long INDICATOR_BLINK_MS = 250;

// ============================================================
// 4) RC / OUTPUT CONFIG
// ============================================================

const int RC_MIN_US     = 1000;
const int RC_NEUTRAL_US = 1500;
const int RC_MAX_US     = 2000;

const int FULL_LEFT_US   = 2000;
const int CENTER_US      = 1500;
const int FULL_RIGHT_US  = 1000;

const int THROTTLE_NEUTRAL_US = 1500;
const int AUTO_THROTTLE_US    = 1542;

const unsigned long BOOT_NEUTRAL_HOLD_MS = 2500;

// ============================================================
// 5) MODE DEFINITIONS
// ============================================================

const uint8_t MODE_MANUAL = 0;
const uint8_t MODE_AUTO   = 1;

const uint8_t APPLIED_MANUAL = 0;
const uint8_t APPLIED_AUTO   = 1;

const uint8_t AUTO_ENTER_STREAK   = 3;
const uint8_t MANUAL_ENTER_STREAK = 1;

// ============================================================
// 6) RADIO PACKET (must match TX exactly)
// ============================================================

static const uint16_t PKT_MAGIC = 0xA55A;
static const uint8_t  PKT_VER   = 3;

struct __attribute__((packed)) ControlPacket {
  uint16_t magic;
  uint8_t  version;
  uint32_t sessionId;
  uint16_t seq;
  int16_t  steerUs;
  int16_t  throttleUs;
  uint8_t  mode;
  uint16_t checksum;
};

// ============================================================
// 7) SENSOR PACKET (must match ESP32-SENSOR exactly)
// ============================================================

struct __attribute__((packed)) SensorPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t seq;
  uint16_t cornerLeft;
  uint16_t midLeft;
  uint16_t frontCenter;
  uint16_t midRight;
  uint16_t cornerRight;
  uint8_t validMask;
  uint8_t checksum;
};

// ============================================================
// 8) SENSOR VALUES DECODED
// ============================================================

struct SensorValues {
  float cl;
  float ml;
  float fc;
  float mr;
  float cr;

  bool clValid;
  bool mlValid;
  bool fcValid;
  bool mrValid;
  bool crValid;
};

SensorValues sv;

// ============================================================
// 9) RADIO / SENSOR TIMEOUTS
// ============================================================

const unsigned long RADIO_TIMEOUT_MS  = 300;
const unsigned long SENSOR_TIMEOUT_MS = 500;

// ============================================================
// 9.1) FORCE INVALID AS MAX DISTANCE
// ============================================================

const bool FORCE_INVALID_AS_MAX_DISTANCE = true;
const float INVALID_AS_DISTANCE_CM = 150.0f;

// ============================================================
// 10) AUTO LOGIC THRESHOLDS
// ============================================================

const float FRONT_TRIGGER_CM   = 70.0f;
const float MID_DIFF_DECIDE_CM = 10.0f;
const float WALL_NEAR_CM       = 22.0f;

// B5: time-based turn-out
const float TURN_OUT_GAIN = 1.10f;

const unsigned long MIN_TURN_IN_MS  = 80;
const unsigned long MAX_TURN_IN_MS  = 1200;
const unsigned long MIN_TURN_OUT_MS = 80;
const unsigned long MAX_TURN_OUT_MS = 1200;

const unsigned long DECIDE_PAUSE_MS = 120;

// ============================================================
// 11) AUTO STATE MACHINE
// ============================================================

const uint8_t AUTO_SAFE_HOLD      = 0;
const uint8_t AUTO_CRUISE         = 1;
const uint8_t AUTO_DECIDE_SIDE    = 2;
const uint8_t AUTO_TURN_LEFT_IN   = 3;
const uint8_t AUTO_TURN_RIGHT_IN  = 4;
const uint8_t AUTO_TURN_LEFT_OUT  = 5;
const uint8_t AUTO_TURN_RIGHT_OUT = 6;

const uint8_t SIDE_NONE  = 0;
const uint8_t SIDE_LEFT  = 1;
const uint8_t SIDE_RIGHT = 2;

// ============================================================
// 12) RUNTIME VARIABLES
// ============================================================

// -------- Radio packet state --------
volatile bool radioPktPending = false;
volatile ControlPacket radioPktBuf;

// Radio stats
uint32_t currentSessionId = 0;
bool radioSessionInitialized = false;
uint16_t lastRadioSeq = 0;
uint32_t radioSessionChanges = 0;
uint32_t radioGoodPackets = 0;
uint32_t radioBadMagic = 0;
uint32_t radioBadVersion = 0;
uint32_t radioBadChecksum = 0;
uint32_t radioBadSize = 0;

unsigned long lastGoodRadioMs = 0;

// -------- Sensor stats --------
uint32_t sensorGoodPackets = 0;
uint32_t sensorBadMagic = 0;
uint32_t sensorBadVersion = 0;
uint32_t sensorBadChecksum = 0;
uint32_t sensorBadSize = 0;

unsigned long lastGoodSensorMs = 0;

// -------- Requested / applied mode --------
uint8_t requestedModeFromTx = MODE_MANUAL;
uint8_t appliedMode = APPLIED_MANUAL;
uint8_t lastAppliedMode = APPLIED_MANUAL;

uint8_t autoModeStreak = 0;
uint8_t manualModeStreak = 0;

// -------- Manual outputs --------
int radioSteerUs = CENTER_US;
int radioThrottleUs = THROTTLE_NEUTRAL_US;

int manualSteerUs = CENTER_US;
int manualThrottleUs = THROTTLE_NEUTRAL_US;

// -------- Auto state + outputs --------
uint8_t autoState = AUTO_SAFE_HOLD;
unsigned long stateEntryMs = 0;

int autoSteerUs = CENTER_US;
int autoThrottleUs = THROTTLE_NEUTRAL_US;

// -------- Final outputs --------
int finalSteerUs = CENTER_US;
int finalThrottleUs = THROTTLE_NEUTRAL_US;

// -------- B5 timing variables --------
unsigned long turnInStartMs = 0;
unsigned long lastTurnInDurationMs = 0;
unsigned long turnOutStartMs = 0;
unsigned long currentTurnOutDurationMs = 0;

// -------- Reverse Helper for MANUAL mode --------
const bool ENABLE_REVERSE_HELPER = false;

const int THR_FORWARD_ENTER_US = 1535;
const int THR_REVERSE_ENTER_US = 1465;

const int THR_NEUTRAL_LOW_US  = 1485;
const int THR_NEUTRAL_HIGH_US = 1515;

const int RH_BRAKE_US = 1350;

const unsigned long RH_BRAKE_HOLD_MS    = 120;
const unsigned long RH_NEUTRAL_HOLD_MS  = 90;
const unsigned long RH_NEUTRAL_RESET_MS = 250;

const uint8_t RH_IDLE         = 0;
const uint8_t RH_BRAKE_HOLD   = 1;
const uint8_t RH_NEUTRAL_HOLD = 2;

uint8_t reverseHelperState = RH_IDLE;
unsigned long reverseHelperStateMs = 0;

int lastManualMotionDir = 0;
int desiredManualThrottleUs = THROTTLE_NEUTRAL_US;
unsigned long lastManualNeutralMs = 0;

// -------- Indicator runtime --------
bool indicatorBlinkPhase = false;
unsigned long indicatorLastToggleMs = 0;

// ============================================================
// 13) FUNCTION PROTOTYPES
// ============================================================

uint16_t checksum16_xor(const uint8_t* data, size_t len_without_checksum);
uint8_t calcSensorChecksum(const SensorPacket& pkt);

bool radioTimedOut();
bool sensorLinkTimedOut();

float decodeDistanceX10(uint16_t raw);
bool isValidBitSet(uint8_t mask, int bitIndex);

const char* modeName(uint8_t mode);
const char* appliedModeName(uint8_t mode);
const char* stateName(uint8_t s);

unsigned long clampUL(unsigned long value, unsigned long low, unsigned long high);
void resetTurnTiming();

void IRAM_ATTR onRadioDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void processPendingRadioPacket();
void processRadioPacket(const ControlPacket& pkt);

void receiveSensorPackets();
void processSensorPacket(const SensorPacket& pkt);

void updateAppliedMode();
void handleModeTransition();

void enterState(uint8_t newState, const char* reason);
uint8_t chooseByPair(float leftValue, bool leftValid,
                     float rightValue, bool rightValid,
                     float minDiffCm);
uint8_t chooseAvoidSide();

void updateAutoStateMachine();
void computeAutoOutputs();
void computeManualOutputs();
void updateFinalOutputs();
void applyFinalOutputs();

int throttleDirFromUs(int us);
const char* reverseHelperStateName(uint8_t s);
void resetReverseHelper();
int applyReverseHelper(int desiredThrottleUs);

void setIndicatorOutputs(bool leftOn, bool rightOn);
void updateIndicators();

void printOneSensor(const char* name, float value, bool valid);
void printB5DebugLine();

// ============================================================
// 14) CHECKSUMS
// ============================================================

uint16_t checksum16_xor(const uint8_t* data, size_t len_without_checksum)
{
  uint16_t x = 0;

  for (size_t i = 0; i < len_without_checksum; i++) {
    x ^= data[i];
    x = (uint16_t)((x << 1) | (x >> 15));
  }

  return x;
}

uint8_t calcSensorChecksum(const SensorPacket& pkt)
{
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t sum = 0;

  for (size_t i = 0; i < sizeof(SensorPacket) - 1; i++) {
    sum += p[i];
  }

  return sum;
}

// ============================================================
// 15) SMALL HELPERS
// ============================================================

unsigned long clampUL(unsigned long value, unsigned long low, unsigned long high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void resetTurnTiming()
{
  turnInStartMs = 0;
  lastTurnInDurationMs = 0;
  turnOutStartMs = 0;
  currentTurnOutDurationMs = 0;
}

// ============================================================
// 16) TIMEOUT HELPERS
// ============================================================

bool radioTimedOut()
{
  return (millis() - lastGoodRadioMs) > RADIO_TIMEOUT_MS;
}

bool sensorLinkTimedOut()
{
  return (millis() - lastGoodSensorMs) > SENSOR_TIMEOUT_MS;
}

// ============================================================
// 17) DECODE HELPERS
// ============================================================

float decodeDistanceX10(uint16_t raw)
{
  return ((float)raw) / 10.0f;
}

bool isValidBitSet(uint8_t mask, int bitIndex)
{
  return (mask & (1 << bitIndex)) != 0;
}

// ============================================================
// 18) NAME HELPERS
// ============================================================

const char* modeName(uint8_t mode)
{
  return (mode == MODE_AUTO) ? "AUTO" : "MANUAL";
}

const char* appliedModeName(uint8_t mode)
{
  return (mode == APPLIED_AUTO) ? "AUTO" : "MANUAL";
}

const char* stateName(uint8_t s)
{
  switch (s) {
    case AUTO_SAFE_HOLD:      return "SAFE_HOLD";
    case AUTO_CRUISE:         return "CRUISE";
    case AUTO_DECIDE_SIDE:    return "DECIDE_SIDE";
    case AUTO_TURN_LEFT_IN:   return "TURN_LEFT_IN";
    case AUTO_TURN_RIGHT_IN:  return "TURN_RIGHT_IN";
    case AUTO_TURN_LEFT_OUT:  return "TURN_LEFT_OUT";
    case AUTO_TURN_RIGHT_OUT: return "TURN_RIGHT_OUT";
    default:                  return "UNKNOWN";
  }
}

// ============================================================
// 19) ESPNOW RECEIVE CALLBACK
// ============================================================

void IRAM_ATTR onRadioDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  if (len != (int)sizeof(ControlPacket)) {
    radioBadSize++;
    return;
  }

  memcpy((void*)&radioPktBuf, incomingData, sizeof(ControlPacket));
  radioPktPending = true;
}

// ============================================================
// 20) RADIO PACKET PROCESSING
// ============================================================

void processPendingRadioPacket()
{
  if (!radioPktPending) return;

  ControlPacket pkt;

  noInterrupts();
  volatile const uint8_t* src = (volatile const uint8_t*)&radioPktBuf;
  uint8_t* dst = (uint8_t*)&pkt;

  for (size_t i = 0; i < sizeof(pkt); i++) {
    dst[i] = src[i];
  }

  radioPktPending = false;
  interrupts();

  processRadioPacket(pkt);
}

void processRadioPacket(const ControlPacket& pkt)
{
  if (pkt.magic != PKT_MAGIC) {
    radioBadMagic++;
    return;
  }

  if (pkt.version != PKT_VER) {
    radioBadVersion++;
    return;
  }

  uint16_t expected = checksum16_xor((const uint8_t*)&pkt,
                                     sizeof(ControlPacket) - sizeof(pkt.checksum));

  if (expected != pkt.checksum) {
    radioBadChecksum++;
    return;
  }

  radioGoodPackets++;
  lastGoodRadioMs = millis();

  if (!radioSessionInitialized || pkt.sessionId != currentSessionId) {
    currentSessionId = pkt.sessionId;
    lastRadioSeq = pkt.seq;
    radioSessionInitialized = true;
    radioSessionChanges++;
  } else {
    lastRadioSeq = pkt.seq;
  }

  radioSteerUs = constrain((int)pkt.steerUs, RC_MIN_US, RC_MAX_US);
  radioThrottleUs = constrain((int)pkt.throttleUs, RC_MIN_US, RC_MAX_US);

  requestedModeFromTx = (pkt.mode == MODE_AUTO) ? MODE_AUTO : MODE_MANUAL;
}

// ============================================================
// 21) SENSOR PACKET PROCESSING
// ============================================================

void receiveSensorPackets()
{
  while (SensorLink.available() >= (int)sizeof(SensorPacket)) {
    SensorPacket pkt;
    int n = SensorLink.readBytes((uint8_t*)&pkt, sizeof(pkt));

    if (n != (int)sizeof(pkt)) {
      sensorBadSize++;
    } else {
      processSensorPacket(pkt);
    }
  }
}

void processSensorPacket(const SensorPacket& pkt)
{
  if (pkt.magic != 0xCAFE) {
    sensorBadMagic++;
    return;
  }

  if (pkt.version != 1) {
    sensorBadVersion++;
    return;
  }

  uint8_t expected = calcSensorChecksum(pkt);
  if (expected != pkt.checksum) {
    sensorBadChecksum++;
    return;
  }

  sensorGoodPackets++;
  lastGoodSensorMs = millis();

  sv.cl = decodeDistanceX10(pkt.cornerLeft);
  sv.ml = decodeDistanceX10(pkt.midLeft);
  sv.fc = decodeDistanceX10(pkt.frontCenter);
  sv.mr = decodeDistanceX10(pkt.midRight);
  sv.cr = decodeDistanceX10(pkt.cornerRight);

  sv.clValid = isValidBitSet(pkt.validMask, 0);
  sv.mlValid = isValidBitSet(pkt.validMask, 1);
  sv.fcValid = isValidBitSet(pkt.validMask, 2);
  sv.mrValid = isValidBitSet(pkt.validMask, 3);
  sv.crValid = isValidBitSet(pkt.validMask, 4);

  // Force INVALID = 150 cm
  if (FORCE_INVALID_AS_MAX_DISTANCE) {
    if (!sv.clValid) {
      sv.cl = INVALID_AS_DISTANCE_CM;
      sv.clValid = true;
    }
    if (!sv.mlValid) {
      sv.ml = INVALID_AS_DISTANCE_CM;
      sv.mlValid = true;
    }
    if (!sv.fcValid) {
      sv.fc = INVALID_AS_DISTANCE_CM;
      sv.fcValid = true;
    }
    if (!sv.mrValid) {
      sv.mr = INVALID_AS_DISTANCE_CM;
      sv.mrValid = true;
    }
    if (!sv.crValid) {
      sv.cr = INVALID_AS_DISTANCE_CM;
      sv.crValid = true;
    }
  }
}

// ============================================================
// 22) MODE APPLICATION
// ============================================================

void updateAppliedMode()
{
  if (radioTimedOut()) {
    autoModeStreak = 0;
    manualModeStreak = 0;
    return;
  }

  if (requestedModeFromTx == MODE_AUTO) {
    autoModeStreak++;
    manualModeStreak = 0;

    if (autoModeStreak >= AUTO_ENTER_STREAK) {
      appliedMode = APPLIED_AUTO;
    }
  } else {
    manualModeStreak++;
    autoModeStreak = 0;

    if (manualModeStreak >= MANUAL_ENTER_STREAK) {
      appliedMode = APPLIED_MANUAL;
    }
  }
}

void handleModeTransition()
{
  if (appliedMode == lastAppliedMode) {
    return;
  }

  resetReverseHelper();
  resetTurnTiming();

  if (appliedMode == APPLIED_AUTO) {
    autoState = AUTO_SAFE_HOLD;
    stateEntryMs = millis();

    autoSteerUs = CENTER_US;
    autoThrottleUs = THROTTLE_NEUTRAL_US;

    Serial.println("[MODE] Enter AUTO -> reset auto state to SAFE_HOLD");
  } else {
    autoState = AUTO_SAFE_HOLD;
    stateEntryMs = millis();

    autoSteerUs = CENTER_US;
    autoThrottleUs = THROTTLE_NEUTRAL_US;

    Serial.println("[MODE] Enter MANUAL -> force auto state to SAFE_HOLD");
  }

  lastAppliedMode = appliedMode;
}

// ============================================================
// 23) AUTO SIDE DECISION
// ============================================================

uint8_t chooseByPair(float leftValue, bool leftValid,
                     float rightValue, bool rightValid,
                     float minDiffCm)
{
  if (leftValid && rightValid) {
    float diff = leftValue - rightValue;

    if (diff >= minDiffCm)  return SIDE_LEFT;
    if (-diff >= minDiffCm) return SIDE_RIGHT;

    return SIDE_NONE;
  }

  if (leftValid && !rightValid) return SIDE_LEFT;
  if (!leftValid && rightValid) return SIDE_RIGHT;

  return SIDE_NONE;
}

uint8_t chooseAvoidSide()
{
  uint8_t side = chooseByPair(sv.ml, sv.mlValid, sv.mr, sv.mrValid, MID_DIFF_DECIDE_CM);
  if (side != SIDE_NONE) {
    return side;
  }

  side = chooseByPair(sv.cl, sv.clValid, sv.cr, sv.crValid, 0.1f);
  return side;
}

// ============================================================
// 24) AUTO STATE MACHINE (B5)
// ============================================================

void enterState(uint8_t newState, const char* reason)
{
  if (autoState != newState) {
    autoState = newState;
    stateEntryMs = millis();

    Serial.print("[STATE] -> ");
    Serial.print(stateName(autoState));
    Serial.print(" | reason: ");
    Serial.println(reason);
  }
}

void updateAutoStateMachine()
{
  if (sensorLinkTimedOut()) {
    resetTurnTiming();
    enterState(AUTO_SAFE_HOLD, "sensor timeout");
  }

  if (autoState == AUTO_SAFE_HOLD) {
    if (!sensorLinkTimedOut() && sv.fcValid) {
      resetTurnTiming();
      enterState(AUTO_CRUISE, "sensor recovered");
    }
  }

  switch (autoState) {
    case AUTO_SAFE_HOLD:
      break;

    case AUTO_CRUISE:
      if (!sv.fcValid) {
        resetTurnTiming();
        enterState(AUTO_SAFE_HOLD, "front invalid in cruise");
        break;
      }

      if (sv.fc <= FRONT_TRIGGER_CM) {
        resetTurnTiming();
        enterState(AUTO_DECIDE_SIDE, "front obstacle detected");
      }
      break;

    case AUTO_DECIDE_SIDE:
      if (!sv.fcValid) {
        resetTurnTiming();
        enterState(AUTO_SAFE_HOLD, "front invalid in decide");
        break;
      }

      if ((millis() - stateEntryMs) >= DECIDE_PAUSE_MS) {
        uint8_t side = chooseAvoidSide();

        if (side == SIDE_LEFT) {
          turnInStartMs = millis();
          lastTurnInDurationMs = 0;
          currentTurnOutDurationMs = 0;
          enterState(AUTO_TURN_LEFT_IN, "left clearer");
        } else if (side == SIDE_RIGHT) {
          turnInStartMs = millis();
          lastTurnInDurationMs = 0;
          currentTurnOutDurationMs = 0;
          enterState(AUTO_TURN_RIGHT_IN, "right clearer");
        } else {
          // Giu state de quan sat tiep
        }
      }
      break;

    case AUTO_TURN_LEFT_IN:
      if (sv.cl <= WALL_NEAR_CM) {
        unsigned long rawTurnIn = millis() - turnInStartMs;
        lastTurnInDurationMs = clampUL(rawTurnIn, MIN_TURN_IN_MS, MAX_TURN_IN_MS);

        unsigned long rawTurnOut = (unsigned long)(lastTurnInDurationMs * TURN_OUT_GAIN);
        currentTurnOutDurationMs = clampUL(rawTurnOut, MIN_TURN_OUT_MS, MAX_TURN_OUT_MS);

        turnOutStartMs = millis();

        enterState(AUTO_TURN_RIGHT_OUT, "CL near wall -> timed right out");
      }
      break;

    case AUTO_TURN_RIGHT_IN:
      if (sv.cr <= WALL_NEAR_CM) {
        unsigned long rawTurnIn = millis() - turnInStartMs;
        lastTurnInDurationMs = clampUL(rawTurnIn, MIN_TURN_IN_MS, MAX_TURN_IN_MS);

        unsigned long rawTurnOut = (unsigned long)(lastTurnInDurationMs * TURN_OUT_GAIN);
        currentTurnOutDurationMs = clampUL(rawTurnOut, MIN_TURN_OUT_MS, MAX_TURN_OUT_MS);

        turnOutStartMs = millis();

        enterState(AUTO_TURN_LEFT_OUT, "CR near wall -> timed left out");
      }
      break;

    case AUTO_TURN_RIGHT_OUT:
      if ((millis() - turnOutStartMs) >= currentTurnOutDurationMs) {
        resetTurnTiming();
        enterState(AUTO_CRUISE, "timed right-out completed");
      }
      break;

    case AUTO_TURN_LEFT_OUT:
      if ((millis() - turnOutStartMs) >= currentTurnOutDurationMs) {
        resetTurnTiming();
        enterState(AUTO_CRUISE, "timed left-out completed");
      }
      break;

    default:
      resetTurnTiming();
      enterState(AUTO_SAFE_HOLD, "unknown state");
      break;
  }
}

// ============================================================
// 25) COMPUTE OUTPUTS
// ============================================================

void computeAutoOutputs()
{
  autoSteerUs = CENTER_US;
  autoThrottleUs = THROTTLE_NEUTRAL_US;

  if (millis() < BOOT_NEUTRAL_HOLD_MS) {
    return;
  }

  if (sensorLinkTimedOut()) {
    return;
  }

  switch (autoState) {
    case AUTO_SAFE_HOLD:
      autoSteerUs = CENTER_US;
      autoThrottleUs = THROTTLE_NEUTRAL_US;
      break;

    case AUTO_CRUISE:
      autoSteerUs = CENTER_US;
      autoThrottleUs = AUTO_THROTTLE_US;
      break;

    case AUTO_DECIDE_SIDE:
      autoSteerUs = CENTER_US;
      autoThrottleUs = THROTTLE_NEUTRAL_US;
      break;

    case AUTO_TURN_LEFT_IN:
      autoSteerUs = FULL_LEFT_US;
      autoThrottleUs = AUTO_THROTTLE_US;
      break;

    case AUTO_TURN_RIGHT_IN:
      autoSteerUs = FULL_RIGHT_US;
      autoThrottleUs = AUTO_THROTTLE_US;
      break;

    case AUTO_TURN_LEFT_OUT:
      autoSteerUs = FULL_LEFT_US;
      autoThrottleUs = AUTO_THROTTLE_US;
      break;

    case AUTO_TURN_RIGHT_OUT:
      autoSteerUs = FULL_RIGHT_US;
      autoThrottleUs = AUTO_THROTTLE_US;
      break;

    default:
      autoSteerUs = CENTER_US;
      autoThrottleUs = THROTTLE_NEUTRAL_US;
      break;
  }
}

// ============================================================
// 26) REVERSE HELPER
// ============================================================

int throttleDirFromUs(int us)
{
  if (us >= THR_FORWARD_ENTER_US) return +1;
  if (us <= THR_REVERSE_ENTER_US) return -1;
  return 0;
}

const char* reverseHelperStateName(uint8_t s)
{
  switch (s) {
    case RH_IDLE:         return "IDLE";
    case RH_BRAKE_HOLD:   return "BRAKE";
    case RH_NEUTRAL_HOLD: return "NEUTRAL";
    default:              return "UNK";
  }
}

void resetReverseHelper()
{
  reverseHelperState = RH_IDLE;
  reverseHelperStateMs = millis();
  lastManualMotionDir = 0;
  lastManualNeutralMs = millis();
}

int applyReverseHelper(int desiredThrottleUs)
{
  unsigned long now = millis();
  int dir = throttleDirFromUs(desiredThrottleUs);

  if (dir == 0) {
    if ((now - lastManualNeutralMs) >= RH_NEUTRAL_RESET_MS &&
        reverseHelperState == RH_IDLE) {
      lastManualMotionDir = 0;
    }
  } else {
    lastManualNeutralMs = now;
  }

  switch (reverseHelperState) {
    case RH_IDLE:
      if (dir > 0) {
        lastManualMotionDir = +1;
        return desiredThrottleUs;
      }

      if (dir == 0) {
        return THROTTLE_NEUTRAL_US;
      }

      if (lastManualMotionDir == +1) {
        reverseHelperState = RH_BRAKE_HOLD;
        reverseHelperStateMs = now;
        return RH_BRAKE_US;
      }

      lastManualMotionDir = -1;
      return desiredThrottleUs;

    case RH_BRAKE_HOLD:
      if (dir >= 0) {
        reverseHelperState = RH_IDLE;
        if (dir > 0) lastManualMotionDir = +1;
        return desiredThrottleUs;
      }

      if ((now - reverseHelperStateMs) < RH_BRAKE_HOLD_MS) {
        return RH_BRAKE_US;
      }

      reverseHelperState = RH_NEUTRAL_HOLD;
      reverseHelperStateMs = now;
      return THROTTLE_NEUTRAL_US;

    case RH_NEUTRAL_HOLD:
      if (dir >= 0) {
        reverseHelperState = RH_IDLE;
        if (dir > 0) lastManualMotionDir = +1;
        return desiredThrottleUs;
      }

      if ((now - reverseHelperStateMs) < RH_NEUTRAL_HOLD_MS) {
        return THROTTLE_NEUTRAL_US;
      }

      reverseHelperState = RH_IDLE;
      lastManualMotionDir = -1;
      return desiredThrottleUs;

    default:
      reverseHelperState = RH_IDLE;
      return THROTTLE_NEUTRAL_US;
  }
}

void computeManualOutputs()
{
  if (radioTimedOut()) {
    manualSteerUs = CENTER_US;
    manualThrottleUs = THROTTLE_NEUTRAL_US;
    resetReverseHelper();
    return;
  }

  manualSteerUs = constrain(radioSteerUs, RC_MIN_US, RC_MAX_US);
  desiredManualThrottleUs = constrain(radioThrottleUs, RC_MIN_US, RC_MAX_US);

  if (ENABLE_REVERSE_HELPER && appliedMode == APPLIED_MANUAL) {
    manualThrottleUs = applyReverseHelper(desiredManualThrottleUs);
  } else {
    manualThrottleUs = desiredManualThrottleUs;
  }

  manualThrottleUs = constrain(manualThrottleUs, RC_MIN_US, RC_MAX_US);
}

void updateFinalOutputs()
{
  finalSteerUs = CENTER_US;
  finalThrottleUs = THROTTLE_NEUTRAL_US;

  if (radioTimedOut()) {
    return;
  }

  if (appliedMode == APPLIED_MANUAL) {
    finalSteerUs = manualSteerUs;
    finalThrottleUs = manualThrottleUs;
    return;
  }

  if (sensorLinkTimedOut()) {
    finalSteerUs = CENTER_US;
    finalThrottleUs = THROTTLE_NEUTRAL_US;
    return;
  }

  finalSteerUs = autoSteerUs;
  finalThrottleUs = autoThrottleUs;
}

// ============================================================
// 27) INDICATORS
// ============================================================

void setIndicatorOutputs(bool leftOn, bool rightOn)
{
  if (INDICATOR_ACTIVE_HIGH) {
    digitalWrite(LEFT_INDICATOR_PIN,  leftOn  ? HIGH : LOW);
    digitalWrite(RIGHT_INDICATOR_PIN, rightOn ? HIGH : LOW);
  } else {
    digitalWrite(LEFT_INDICATOR_PIN,  leftOn  ? LOW : HIGH);
    digitalWrite(RIGHT_INDICATOR_PIN, rightOn ? LOW : HIGH);
  }
}

void updateIndicators()
{
  unsigned long now = millis();

  if (now - indicatorLastToggleMs >= INDICATOR_BLINK_MS) {
    indicatorLastToggleMs = now;
    indicatorBlinkPhase = !indicatorBlinkPhase;
  }

  bool leftOn = false;
  bool rightOn = false;

  if (appliedMode == APPLIED_MANUAL) {
    int steerError = manualSteerUs - CENTER_US;

    if (steerError >= INDICATOR_STEER_THRESHOLD_US) {
      leftOn = indicatorBlinkPhase;
    } else if (steerError <= -INDICATOR_STEER_THRESHOLD_US) {
      rightOn = indicatorBlinkPhase;
    }
  }

  setIndicatorOutputs(leftOn, rightOn);
}

// ============================================================
// 28) OPTIONAL REAL PWM OUTPUT
// ============================================================

void applyFinalOutputs()
{
  if (!ENABLE_PWM_OUTPUT) {
    return;
  }

  int steerToWrite = finalSteerUs;

  if (REVERSE_STEER_SERVO) {
    steerToWrite = RC_MIN_US + RC_MAX_US - finalSteerUs;
  }

  steerToWrite = constrain(steerToWrite, RC_MIN_US, RC_MAX_US);

  steerServo.writeMicroseconds(steerToWrite);
  escServo.writeMicroseconds(finalThrottleUs);
}

// ============================================================
// 29) DEBUG LOG
// ============================================================

void printOneSensor(const char* name, float value, bool valid)
{
  Serial.print(name);
  Serial.print(":");
  if (valid) {
    Serial.print(value, 1);
  } else {
    Serial.print("INV");
  }
}

void printB5DebugLine()
{
  static unsigned long lastLogMs = 0;
  const unsigned long LOG_PERIOD_MS = 150;
  unsigned long now = millis();

  if (now - lastLogMs < LOG_PERIOD_MS) return;
  lastLogMs = now;

  Serial.print("[B5] req=");
  Serial.print(modeName(requestedModeFromTx));

  Serial.print(" | applied=");
  Serial.print(appliedModeName(appliedMode));

  Serial.print(" | autoState=");
  Serial.print(stateName(autoState));

  Serial.print(" | rTO=");
  Serial.print(radioTimedOut() ? "Y" : "N");

  Serial.print(" | sTO=");
  Serial.print(sensorLinkTimedOut() ? "Y" : "N");

  Serial.print(" | manSt=");
  Serial.print(manualSteerUs);
  Serial.print(" manTh=");
  Serial.print(manualThrottleUs);

  Serial.print(" | desManTh=");
  Serial.print(desiredManualThrottleUs);

  Serial.print(" | revH=");
  Serial.print(reverseHelperStateName(reverseHelperState));

  Serial.print(" | autoSt=");
  Serial.print(autoSteerUs);
  Serial.print(" autoTh=");
  Serial.print(autoThrottleUs);

  Serial.print(" | tIn=");
  Serial.print(lastTurnInDurationMs);
  Serial.print(" tOut=");
  Serial.print(currentTurnOutDurationMs);

  Serial.print(" | finalSt=");
  Serial.print(finalSteerUs);
  Serial.print(" finalTh=");
  Serial.print(finalThrottleUs);

  Serial.print(" | radioGood=");
  Serial.print(radioGoodPackets);
  Serial.print(" sensorGood=");
  Serial.print(sensorGoodPackets);

  Serial.print(" | ");
  printOneSensor("CL", sv.cl, sv.clValid); Serial.print(" | ");
  printOneSensor("ML", sv.ml, sv.mlValid); Serial.print(" | ");
  printOneSensor("FC", sv.fc, sv.fcValid); Serial.print(" | ");
  printOneSensor("MR", sv.mr, sv.mrValid); Serial.print(" | ");
  printOneSensor("CR", sv.cr, sv.crValid);

  Serial.println();
}

// ============================================================
// 30) SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(300);

  SensorLink.begin(SENSOR_UART_BAUD, SERIAL_8N1, SENSOR_UART_RX_PIN, -1);

  pinMode(LEFT_INDICATOR_PIN, OUTPUT);
  pinMode(RIGHT_INDICATOR_PIN, OUTPUT);
  setIndicatorOutputs(false, false);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.println("=== B5 RX START (MANUAL/AUTO + Indicator + INVALID=150) ===");
  Serial.print("RX MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init() FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onRadioDataRecv);

  if (ENABLE_PWM_OUTPUT) {
    steerServo.setPeriodHertz(50);
    escServo.setPeriodHertz(50);

    steerServo.attach(STEER_SERVO_PIN, 1000, 2000);
    escServo.attach(ESC_PWM_PIN, 1000, 2000);

    steerServo.writeMicroseconds(CENTER_US);
    escServo.writeMicroseconds(THROTTLE_NEUTRAL_US);
  }

  appliedMode = APPLIED_MANUAL;
  requestedModeFromTx = MODE_MANUAL;
  lastAppliedMode = appliedMode;

  autoState = AUTO_SAFE_HOLD;
  stateEntryMs = millis();

  radioSteerUs = CENTER_US;
  radioThrottleUs = THROTTLE_NEUTRAL_US;

  manualSteerUs = CENTER_US;
  manualThrottleUs = THROTTLE_NEUTRAL_US;

  autoSteerUs = CENTER_US;
  autoThrottleUs = THROTTLE_NEUTRAL_US;

  finalSteerUs = CENTER_US;
  finalThrottleUs = THROTTLE_NEUTRAL_US;

  resetReverseHelper();
  resetTurnTiming();

  indicatorBlinkPhase = false;
  indicatorLastToggleMs = millis();

  Serial.print("PWM output enabled? ");
  Serial.println(ENABLE_PWM_OUTPUT ? "YES" : "NO");
}

// ============================================================
// 31) LOOP
// ============================================================

void loop()
{
  // 1) Xu ly packet radio pending tu callback
  processPendingRadioPacket();

  // 2) Nhan packet sensor tu UART
  receiveSensorPackets();

  // 3) Cap nhat mode duoc ap dung
  updateAppliedMode();

  // 4) Xu ly chuyen mode MANUAL <-> AUTO
  handleModeTransition();

  // 5) Tinh output manual
  computeManualOutputs();

  // 6) Chi cho auto chay khi dang o AUTO
  if (appliedMode == APPLIED_AUTO) {
    updateAutoStateMachine();
    computeAutoOutputs();
  } else {
    autoState = AUTO_SAFE_HOLD;
    autoSteerUs = CENTER_US;
    autoThrottleUs = THROTTLE_NEUTRAL_US;
    resetTurnTiming();
  }

  // 7) Chon output cuoi
  updateFinalOutputs();

  // 7.1) Cap nhat xi nhan
  updateIndicators();

  // 8) Neu bat actuator that thi moi phat PWM
  applyFinalOutputs();

  // 9) Log monitor
  printB5DebugLine();
}