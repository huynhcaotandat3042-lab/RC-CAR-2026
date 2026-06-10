#include <Arduino.h>
#include <Wire.h>                 // I2C cho LCD
#include <LiquidCrystal_I2C.h>    // LCD2004 I2C
#include <WiFi.h>                 // Wi-Fi STA mode cho ESP-NOW
#include <esp_now.h>              // ESP-NOW
#include <esp_wifi.h>             // fix channel
#include <esp_system.h>           // esp_random() để tạo sessionId
#include <SimpleKalmanFilter.h>   // lọc ADC joystick

// ==========================================================
// 0) CẤU HÌNH LCD
// ==========================================================

// Địa chỉ LCD đã scan được
const uint8_t LCD_ADDR = 0x27;

// LCD 20 cột, 4 dòng
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

// Chân I2C mặc định ESP32
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// Chu kỳ update LCD
const uint32_t LCD_UPDATE_MS = 200;

// ==========================================================
// 1) CẤU HÌNH JOYSTICK + NÚT TRIM + CÔNG TẮC MODE
// ==========================================================

// Steer joystick
#define STEER_JOY_PIN 34

// Throttle joystick
#define THROTTLE_JOY_PIN 35

// 2 nút trim, dùng INPUT_PULLUP => thả = HIGH, bấm = LOW
const int TRIM_PLUS_PIN  = 25;
const int TRIM_MINUS_PIN = 27;

// Công tắc 2 vị trí chọn mode
// Gợi ý wiring:
// - 1 chân switch -> GPIO32
// - 1 chân switch -> GND
// - pinMode(INPUT_PULLUP)
// Khi switch đóng => LOW
const int MODE_SWITCH_PIN = 32;

// ----------------------------------------------------------
// CALIBRATION JOYSTICK GIMBAL
// ----------------------------------------------------------
// KHÔNG còn giả định joystick chạy đủ 0..4095.
// Hãy đo từng joystick thực tế rồi nhập vào đây.
//
// Cách đo:
// - Thả tay joystick 5-10 lần, lấy giá trị center ổn định.
// - Gạt hết trái/phải hoặc tiến/lùi nhiều lần, lấy min/max thực tế.
// - Không cần min = 0, max = 4095. Ví dụ 250..4050 vẫn bình thường.
//
// Lưu ý:
// - STEER_ADC_MIN phải nhỏ hơn STEER_ADC_CENTER.
// - STEER_ADC_MAX phải lớn hơn STEER_ADC_CENTER.
// - THROTTLE_ADC_MIN phải nhỏ hơn THROTTLE_ADC_CENTER.
// - THROTTLE_ADC_MAX phải lớn hơn THROTTLE_ADC_CENTER.

const int STEER_ADC_MIN    = 90;   // sửa theo gimbal lái thực tế
const int STEER_ADC_CENTER = 1900;  // center lái hiện tại của bạn
const int STEER_ADC_MAX    = 3800;  // sửa theo gimbal lái thực tế

const int THROTTLE_ADC_MIN    = 160;   // sửa theo gimbal ga thực tế
const int THROTTLE_ADC_CENTER = 1950;  // center ga hiện tại của bạn
const int THROTTLE_ADC_MAX    = 4000;  // sửa theo gimbal ga thực tế

// Deadzone ADC cho từng joystick
// Nếu thả ga mà xe tự bò nhẹ => tăng THROTTLE_DEADZONE_ADC lên 60/80.
// Nếu lái quanh giữa bị ì quá => giảm STEER_DEADZONE_ADC xuống 20/30.
const int STEER_DEADZONE_ADC    = 50;
const int THROTTLE_DEADZONE_ADC = 60;

// Kalman filter
// Theo baseline sơ tuyển: steer dùng Kalman, throttle vẫn đọc raw cho output.
SimpleKalmanFilter kfSteer(2, 2, 0.01);
SimpleKalmanFilter kfThrottle(2, 2, 0.01);

// Dải microseconds kiểu RC
const int RC_MIN_US     = 1000;
const int RC_MAX_US     = 2000;
const int RC_NEUTRAL_US = 1500;

// Trim chỉ áp cho steer
const int TRIM_STEP_US  = 10;
const int TRIM_LIMIT_US = 210;

// Debounce nút trim
const uint32_t DEBOUNCE_MS = 40;

// Debounce công tắc mode
const uint32_t MODE_DEBOUNCE_MS = 30;

// Chu kỳ xử lý joystick + gửi radio
const uint32_t TX_UPDATE_MS = 20;   // 50Hz

// ==========================================================
// 2) CẤU HÌNH HƯỚNG JOYSTICK
// ==========================================================

// STEER:
// Baseline cũ đang dùng: ADC cao hơn center => steerUs > 1500.
// Nếu sau khi thay gimbal thấy trái/phải bị ngược ở MANUAL,
// đổi true <-> false ở biến này trước khi đụng RX.
const bool STEER_ADC_HIGHER_WHEN_RIGHT = true;

// THROTTLE:
// Bạn muốn "đẩy joystick lên = tiến".
// Baseline sơ tuyển của bạn đang là false.
// Nếu test thực tế thấy tiến/lùi bị ngược, đổi FALSE <-> TRUE.
const bool THROTTLE_ADC_HIGHER_WHEN_FORWARD = true;

// ==========================================================
// 3) CẤU HÌNH ESP-NOW
// ==========================================================

// TX và RX phải cùng channel
static const uint8_t ESPNOW_CHANNEL = 6;

// MAC RX hiện đang dùng trong code thật của bạn
uint8_t rxMac[] = { 0xB0, 0xCB, 0xD8, 0x8B, 0x6E, 0x14 };

// ==========================================================
// 4) RADIO MODE
// ==========================================================

enum RadioMode : uint8_t {
  MODE_MANUAL = 0,
  MODE_AUTO   = 1
};

// Nếu switch đang LOW -> AUTO, HIGH -> MANUAL
// Nếu thực tế bị ngược, chỉ cần đảo logic trong readModeSwitchRaw()
uint8_t lastRawMode = MODE_MANUAL;
uint8_t stableMode = MODE_MANUAL;
uint32_t modeLastChangeMs = 0;

// ==========================================================
// 5) ĐỊNH NGHĨA PACKET RADIO
// ==========================================================

static const uint16_t PKT_MAGIC = 0xA55A;
static const uint8_t  PKT_VER   = 3;

#pragma pack(push, 1)
struct ControlPacket {
  uint16_t magic;        // dấu hiệu nhận biết đúng packet
  uint8_t  version;      // version protocol
  uint32_t sessionId;    // ID phiên TX, đổi mỗi lần TX boot
  uint16_t seq;          // số thứ tự packet trong cùng session
  int16_t  steerUs;      // lệnh lái
  int16_t  throttleUs;   // lệnh ga
  uint8_t  mode;         // 0 = MANUAL, 1 = AUTO
  uint16_t checksum;     // checksum bắt lỗi packet
};
#pragma pack(pop)

// ==========================================================
// 6) BIẾN TRẠNG THÁI
// ==========================================================

// ADC steer
int rawSteerValue = 0;
float filteredSteerValue = 0;

// ADC throttle
int rawThrottleValue = 0;
float filteredThrottleValue = 0;

// Kết quả RC steer
int steerBaseUs = RC_NEUTRAL_US;   // steer trước trim
int steerOutUs  = RC_NEUTRAL_US;   // steer sau trim

// Kết quả RC throttle
int throttleOutUs = RC_NEUTRAL_US; // throttle sau map

// Trim steer
int trimOffsetUs = 0;

// Session ID TX
uint32_t txSessionId = 0;

// Seq packet
uint16_t txSeq = 0;

// Thống kê gửi
uint32_t sendOk = 0;
uint32_t sendFail = 0;

// ==========================================================
// 7) HÀM CHECKSUM
// ==========================================================

uint16_t checksum16_xor(const uint8_t* data, size_t len_without_checksum) {
  uint16_t x = 0;

  for (size_t i = 0; i < len_without_checksum; i++) {
    x ^= data[i];
    x = (uint16_t)((x << 1) | (x >> 15));
  }

  return x;
}

// ==========================================================
// 8) HÀM MAP ADC ĐÃ CALIBRATE MIN/CENTER/MAX -> RC US
// ==========================================================

bool isAxisCalibrationValid(int adcMin, int adcCenter, int adcMax, int deadband) {
  if (adcMin >= adcCenter) return false;
  if (adcCenter >= adcMax) return false;
  if ((adcCenter - deadband) <= adcMin) return false;
  if ((adcCenter + deadband) >= adcMax) return false;
  return true;
}

int mapCalibratedAxisToUs(int adc,
                          int adcMin,
                          int adcCenter,
                          int adcMax,
                          int deadband,
                          bool higherMeansPositive) {
  // Nếu nhập sai calibration thì trả neutral để an toàn hơn là map lỗi.
  if (!isAxisCalibrationValid(adcMin, adcCenter, adcMax, deadband)) {
    return RC_NEUTRAL_US;
  }

  adc = constrain(adc, adcMin, adcMax);

  const int lowEdge  = adcCenter - deadband;
  const int highEdge = adcCenter + deadband;

  // Trong deadzone => neutral tuyệt đối
  if (adc >= lowEdge && adc <= highEdge) {
    return RC_NEUTRAL_US;
  }

  long offsetUs = 0;

  if (adc > highEdge) {
    // Nửa trên: highEdge -> adcMax tương ứng 0 -> 500us
    long denom = adcMax - highEdge;
    offsetUs = (long)(adc - highEdge) * (RC_MAX_US - RC_NEUTRAL_US) / denom;

    if (higherMeansPositive) {
      return constrain((int)(RC_NEUTRAL_US + offsetUs), RC_MIN_US, RC_MAX_US);
    } else {
      return constrain((int)(RC_NEUTRAL_US - offsetUs), RC_MIN_US, RC_MAX_US);
    }
  } else {
    // Nửa dưới: lowEdge -> adcMin tương ứng 0 -> 500us
    long denom = lowEdge - adcMin;
    offsetUs = (long)(lowEdge - adc) * (RC_NEUTRAL_US - RC_MIN_US) / denom;

    if (higherMeansPositive) {
      return constrain((int)(RC_NEUTRAL_US - offsetUs), RC_MIN_US, RC_MAX_US);
    } else {
      return constrain((int)(RC_NEUTRAL_US + offsetUs), RC_MIN_US, RC_MAX_US);
    }
  }
}

void printAxisCalibrationStatus() {
  Serial.println("\n=== Joystick calibration ===");

  Serial.print("STEER min/center/max/deadzone = ");
  Serial.print(STEER_ADC_MIN); Serial.print("/");
  Serial.print(STEER_ADC_CENTER); Serial.print("/");
  Serial.print(STEER_ADC_MAX); Serial.print("/");
  Serial.print(STEER_DEADZONE_ADC);
  Serial.print(" -> ");
  Serial.println(isAxisCalibrationValid(STEER_ADC_MIN, STEER_ADC_CENTER, STEER_ADC_MAX, STEER_DEADZONE_ADC) ? "OK" : "INVALID");

  Serial.print("THROTTLE min/center/max/deadzone = ");
  Serial.print(THROTTLE_ADC_MIN); Serial.print("/");
  Serial.print(THROTTLE_ADC_CENTER); Serial.print("/");
  Serial.print(THROTTLE_ADC_MAX); Serial.print("/");
  Serial.print(THROTTLE_DEADZONE_ADC);
  Serial.print(" -> ");
  Serial.println(isAxisCalibrationValid(THROTTLE_ADC_MIN, THROTTLE_ADC_CENTER, THROTTLE_ADC_MAX, THROTTLE_DEADZONE_ADC) ? "OK" : "INVALID");
}

// ==========================================================
// 9) DEBOUNCE NÚT (POLLING)
// ==========================================================

struct DebouncedButton {
  int pin = -1;

  bool stableState = HIGH;
  bool lastReading = HIGH;
  uint32_t lastChangeMs = 0;

  void begin(int p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
    stableState = digitalRead(pin);
    lastReading = stableState;
    lastChangeMs = millis();
  }

  bool fell(uint32_t nowMs, uint32_t debounceMs) {
    bool reading = digitalRead(pin);

    if (reading != lastReading) {
      lastReading = reading;
      lastChangeMs = nowMs;
    }

    if ((nowMs - lastChangeMs) >= debounceMs && stableState != lastReading) {
      stableState = lastReading;
      if (stableState == LOW) return true;
    }

    return false;
  }
};

DebouncedButton btnPlus;
DebouncedButton btnMinus;

// ==========================================================
// 10) MODE SWITCH
// ==========================================================

uint8_t readModeSwitchRaw()
{
  // LOW = AUTO, HIGH = MANUAL
  if (digitalRead(MODE_SWITCH_PIN) == LOW) {
    return MODE_AUTO;
  } else {
    return MODE_MANUAL;
  }
}

void updateModeSwitchDebounce(uint32_t nowMs)
{
  uint8_t currentRawMode = readModeSwitchRaw();

  if (currentRawMode != lastRawMode) {
    lastRawMode = currentRawMode;
    modeLastChangeMs = nowMs;
  }

  if ((nowMs - modeLastChangeMs) >= MODE_DEBOUNCE_MS) {
    stableMode = currentRawMode;
  }
}

const char* modeName(uint8_t mode)
{
  return (mode == MODE_AUTO) ? "AUTO" : "MANUAL";
}

// ==========================================================
// 11) CALLBACK KHI GỬI XONG
// ==========================================================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    sendOk++;
  } else {
    sendFail++;
  }
}

// ==========================================================
// 12) SETUP
// ==========================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // ---------- (A) INIT LCD ----------
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  lcd.init();
  lcd.backlight();

  // In label cố định 1 lần để tránh nháy
  lcd.setCursor(0, 0); lcd.print("RC CAR 2026 V4.2");
  lcd.setCursor(0, 1); lcd.print("strB:");
  lcd.setCursor(0, 2); lcd.print("trim:");
  lcd.setCursor(0, 3); lcd.print("s/t:");

  // ---------- (B) INIT NÚT ----------
  btnPlus.begin(TRIM_PLUS_PIN);
  btnMinus.begin(TRIM_MINUS_PIN);

  // ---------- (C) INIT CÔNG TẮC MODE ----------
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
  lastRawMode = readModeSwitchRaw();
  stableMode = lastRawMode;
  modeLastChangeMs = millis();

  // ---------- (D) INIT WIFI + ESP-NOW ----------
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n=== ESP-NOW V4.2 TX (Gimbal Calibrated Min/Center/Max) ===");
  Serial.print("TX MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("Channel: "); Serial.println(ESPNOW_CHANNEL);

  printAxisCalibrationStatus();

  // Tạo sessionId mới mỗi lần TX boot
  txSessionId = esp_random();
  if (txSessionId == 0) {
    txSessionId = 1;
  }

  Serial.print("TX sessionId: 0x");
  Serial.println(txSessionId, HEX);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init() FAILED");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onDataSent);

  // Add peer RX để gửi unicast
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, rxMac, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("esp_now_add_peer() FAILED");
    while (true) delay(1000);
  }

  Serial.print("Initial mode: ");
  Serial.println(modeName(stableMode));

  Serial.println("TX ready. Sending steer + throttle + mode packets...");
}

// ==========================================================
// 13) LOOP
// ==========================================================

void loop() {
  uint32_t now = millis();

  // ---------- (A) XỬ LÝ NÚT TRIM CHO STEER ----------
  if (btnPlus.fell(now, DEBOUNCE_MS)) {
    trimOffsetUs += TRIM_STEP_US;
    if (trimOffsetUs > TRIM_LIMIT_US) trimOffsetUs = TRIM_LIMIT_US;
  }

  if (btnMinus.fell(now, DEBOUNCE_MS)) {
    trimOffsetUs -= TRIM_STEP_US;
    if (trimOffsetUs < -TRIM_LIMIT_US) trimOffsetUs = -TRIM_LIMIT_US;
  }

  // ---------- (B) CẬP NHẬT CÔNG TẮC MODE ----------
  updateModeSwitchDebounce(now);

  // ---------- (C) UPDATE JOYSTICK + TẠO PACKET + GỬI ----------
  static uint32_t lastTxMs = 0;
  if (now - lastTxMs >= TX_UPDATE_MS) {
    lastTxMs += TX_UPDATE_MS;

    // ===== 1) Đọc steer joystick =====
    rawSteerValue = analogRead(STEER_JOY_PIN);
    filteredSteerValue = kfSteer.updateEstimate(rawSteerValue);

    int steerFiltInt = (int)(filteredSteerValue + 0.5f);

    // Steer vẫn dùng Kalman như baseline cũ, nhưng map theo min/center/max mới.
    steerBaseUs = mapCalibratedAxisToUs(steerFiltInt,
                                        STEER_ADC_MIN,
                                        STEER_ADC_CENTER,
                                        STEER_ADC_MAX,
                                        STEER_DEADZONE_ADC,
                                        STEER_ADC_HIGHER_WHEN_RIGHT);

    // Cộng trim cho steer
    steerOutUs = steerBaseUs + trimOffsetUs;
    steerOutUs = constrain(steerOutUs, RC_MIN_US, RC_MAX_US);

    // ===== 2) Đọc throttle joystick =====
    rawThrottleValue = analogRead(THROTTLE_JOY_PIN);
    filteredThrottleValue = kfThrottle.updateEstimate(rawThrottleValue);

    // Theo baseline sơ tuyển: throttle output dùng raw để phản ứng nhanh.
    int throttleInputInt = rawThrottleValue;

    throttleOutUs = mapCalibratedAxisToUs(throttleInputInt,
                                          THROTTLE_ADC_MIN,
                                          THROTTLE_ADC_CENTER,
                                          THROTTLE_ADC_MAX,
                                          THROTTLE_DEADZONE_ADC,
                                          THROTTLE_ADC_HIGHER_WHEN_FORWARD);

    throttleOutUs = constrain(throttleOutUs, RC_MIN_US, RC_MAX_US);

    // ===== 3) Tạo packet =====
    ControlPacket pkt;
    pkt.magic      = PKT_MAGIC;
    pkt.version    = PKT_VER;
    pkt.sessionId  = txSessionId;
    pkt.seq        = txSeq++;
    pkt.steerUs    = (int16_t)steerOutUs;
    pkt.throttleUs = (int16_t)throttleOutUs;
    pkt.mode       = stableMode;

    pkt.checksum = checksum16_xor((const uint8_t*)&pkt,
                                  sizeof(ControlPacket) - sizeof(pkt.checksum));

    // ===== 4) Gửi unicast =====
    esp_err_t r = esp_now_send(rxMac, (uint8_t*)&pkt, sizeof(pkt));

    if (r != ESP_OK) {
      sendFail++;
    }

    // ===== 5) Log ra Serial =====
    Serial.print("sid:0x");      Serial.print(pkt.sessionId, HEX);
    Serial.print(" seq:");       Serial.print(pkt.seq);
    Serial.print(" mode:");      Serial.print(modeName(pkt.mode));
    Serial.print(" strUs:");     Serial.print(pkt.steerUs);
    Serial.print(" thrUs:");     Serial.print(pkt.throttleUs);
    Serial.print(" ok:");        Serial.print(sendOk);
    Serial.print(" fail:");      Serial.print(sendFail);
    Serial.print(" rawStr:");    Serial.print(rawSteerValue);
    Serial.print(" filtStr:");   Serial.print(steerFiltInt);
    Serial.print(" rawThr:");    Serial.print(rawThrottleValue);
    Serial.print(" filtThr:");   Serial.print((int)(filteredThrottleValue + 0.5f));
    Serial.println();
  }

  // ---------- (D) UPDATE LCD ----------
  static uint32_t lastLcdMs = 0;
  if (now - lastLcdMs >= LCD_UPDATE_MS) {
    lastLcdMs += LCD_UPDATE_MS;

    char buf[21];

    // Dòng 1: mode ở góc phải
    lcd.setCursor(17, 0);
    lcd.print(stableMode == MODE_AUTO ? "AUT" : "MAN");

    // Dòng 2: steer base
    snprintf(buf, sizeof(buf), "%4d       ", steerBaseUs);
    lcd.setCursor(5, 1);
    lcd.print(buf);

    // Dòng 3: trim steer
    snprintf(buf, sizeof(buf), "%+4d      ", trimOffsetUs);
    lcd.setCursor(5, 2);
    lcd.print(buf);

    // Dòng 4: steerOut / throttleOut
    snprintf(buf, sizeof(buf), "%4d/%4d ", steerOutUs, throttleOutUs);
    lcd.setCursor(4, 3);
    lcd.print(buf);
  }

  // Không dùng delay() để loop phản ứng nhanh
}
