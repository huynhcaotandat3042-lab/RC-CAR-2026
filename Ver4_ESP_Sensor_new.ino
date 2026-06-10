/*
  ============================================================
  Version B2.1 - ESP32-SENSOR
  ------------------------------------------------------------
  Muc tieu:
  - Doc 5 cam bien sieu am theo thu tu
  - Gui packet UART sang ESP32-RX
  - Tang toc do cap nhat
  - Them loc median3
  - Gioi han gia tri toi da de tranh "qua xa vo nghia"

  Giu nguyen packet protocol cua B2 -> RX khong can doi code
  ============================================================
*/

#include <HardwareSerial.h>

// ============================================================
// 1) DAT TEN LOGIC CHO 5 CAM BIEN
// ============================================================
// S1 = CORNER_LEFT   (goc trai)
// S2 = MID_LEFT      (giua trai)
// S3 = FRONT_CENTER  (giua)
// S4 = MID_RIGHT     (giua phai)
// S5 = CORNER_RIGHT  (goc phai)

// ============================================================
// 2) GPIO CONFIG
// ============================================================

// ---------- TRIG pins ----------
const int S1_TRIG = 13;
const int S2_TRIG = 14;
const int S3_TRIG = 15;
const int S4_TRIG = 16;
const int S5_TRIG = 17;

// ---------- ECHO pins ----------
const int S1_ECHO = 18;
const int S2_ECHO = 19;
const int S3_ECHO = 21;
const int S4_ECHO = 22;
const int S5_ECHO = 23;

// ---------- UART TX sang RX board ----------
const int SENSOR_UART_TX_PIN = 32;
const int SENSOR_UART_BAUD   = 115200;

HardwareSerial SensorLink(2);

// ============================================================
// 3) THAM SO DO KHOANG CACH
// ============================================================

// Giam timeout xuong de he thong phan ung nhanh hon
const unsigned long ECHO_TIMEOUT_US = 10000UL;

// Toc do am thanh xap xi trong khong khi
const float SOUND_SPEED_CM_PER_US = 0.0343f;

// Nghi ngan giua 2 cam bien de giam nguy co nghe lan nhau
const unsigned long INTER_SENSOR_DELAY_MS = 3;

// Delay cuoi vong lap de tang toc do cap nhat
const unsigned long LOOP_DELAY_MS = 0;

// Gioi han logic toi da
// Boi canh bai toan cua ban khong can giu gia tri qua xa nhu 4m
const float MAX_DISTANCE_CM = 150.0f;

// Co the giu mot muc nho de tranh gia tri sat 0 khong thuc te
const float MIN_DISTANCE_CM = 2.0f;

// So mau cho median
const int WINDOW_SIZE = 3;

// ============================================================
// 4) SO CAM BIEN
// ============================================================

const int NUM_SENSORS = 5;

enum SensorIndex {
  IDX_CL = 0,
  IDX_ML = 1,
  IDX_FC = 2,
  IDX_MR = 3,
  IDX_CR = 4
};

const char* SENSOR_NAMES[NUM_SENSORS] = {
  "CL", "ML", "FC", "MR", "CR"
};

const int TRIG_PINS[NUM_SENSORS] = {
  S1_TRIG, S2_TRIG, S3_TRIG, S4_TRIG, S5_TRIG
};

const int ECHO_PINS[NUM_SENSORS] = {
  S1_ECHO, S2_ECHO, S3_ECHO, S4_ECHO, S5_ECHO
};

// ============================================================
// 5) STRUCTS
// ============================================================

struct SensorReading {
  float distanceCm;
  bool valid;
};

struct SensorPacket {
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
// 6) BO NHO PHUC VU WINDOW MEDIAN3
// ============================================================

// Luu 3 gia tri gan nhat cho tung cam bien
float historyValues[NUM_SENSORS][WINDOW_SIZE];

// Luu trang thai valid cho tung gia tri
bool historyValid[NUM_SENSORS][WINDOW_SIZE];

// Gia tri da loc hien tai
SensorReading filteredReadings[NUM_SENSORS];

// So thu tu packet
uint8_t packetSeq = 0;

// ============================================================
// 7) HAM PHU TRO
// ============================================================

float clampDistance(float cm)
{
  if (cm < MIN_DISTANCE_CM) cm = MIN_DISTANCE_CM;
  if (cm > MAX_DISTANCE_CM) cm = MAX_DISTANCE_CM;
  return cm;
}

void initHistory()
{
  for (int i = 0; i < NUM_SENSORS; i++) {
    for (int j = 0; j < WINDOW_SIZE; j++) {
      historyValues[i][j] = 0.0f;
      historyValid[i][j] = false;
    }

    filteredReadings[i].distanceCm = -1.0f;
    filteredReadings[i].valid = false;
  }
}

float medianOf3(float a, float b, float c)
{
  // Cach kinh dien tim median cua 3 so
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================
// 8) DOC 1 CAM BIEN RAW
// ============================================================

SensorReading readRawSensor(int trigPin, int echoPin)
{
  SensorReading result;
  result.distanceCm = -1.0f;
  result.valid = false;

  // Dam bao TRIG o LOW truoc khi ban xung
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Ban xung trigger 10 us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Do do rong xung HIGH
  unsigned long echoTime = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  // Timeout -> invalid
  if (echoTime == 0) {
    return result;
  }

  float distance = (echoTime * SOUND_SPEED_CM_PER_US) / 2.0f;

  // Clamp ngay tai node sensor
  distance = clampDistance(distance);

  result.distanceCm = distance;
  result.valid = true;
  return result;
}

// ============================================================
// 9) CAP NHAT WINDOW 3 MAU GAN NHAT
// ============================================================

void pushHistory(int sensorIndex, SensorReading raw)
{
  // Dich trai: [1] -> [0], [2] -> [1]
  historyValues[sensorIndex][0] = historyValues[sensorIndex][1];
  historyValues[sensorIndex][1] = historyValues[sensorIndex][2];

  historyValid[sensorIndex][0] = historyValid[sensorIndex][1];
  historyValid[sensorIndex][1] = historyValid[sensorIndex][2];

  // Chen mau moi vao cuoi
  historyValues[sensorIndex][2] = raw.distanceCm;
  historyValid[sensorIndex][2] = raw.valid;
}

// ============================================================
// 10) TINH GIA TRI DA LOC TU WINDOW
// ============================================================

SensorReading computeFilteredFromHistory(int sensorIndex)
{
  SensorReading out;
  out.distanceCm = -1.0f;
  out.valid = false;

  float validValues[WINDOW_SIZE];
  int countValid = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (historyValid[sensorIndex][i]) {
      validValues[countValid++] = historyValues[sensorIndex][i];
    }
  }

  // Khong co gia tri hop le nao
  if (countValid == 0) {
    return out;
  }

  // Du 3 mau hop le -> dung median3 dung nghia
  if (countValid == 3) {
    out.distanceCm = medianOf3(validValues[0], validValues[1], validValues[2]);
    out.valid = true;
    return out;
  }

  // Chi co 2 mau hop le -> lay trung binh 2 mau hop le
  // (day la fallback de khong bi qua "mong")
  if (countValid == 2) {
    out.distanceCm = (validValues[0] + validValues[1]) * 0.5f;
    out.valid = true;
    return out;
  }

  // Chi co 1 mau hop le -> dung tam mau do
  out.distanceCm = validValues[0];
  out.valid = true;
  return out;
}

// ============================================================
// 11) DOC TAT CA CAM BIEN THEO THU TU + CAP NHAT LOC
// ============================================================

void readAllSensorsSequentiallyAndFilter()
{
  for (int i = 0; i < NUM_SENSORS; i++) {
    SensorReading raw = readRawSensor(TRIG_PINS[i], ECHO_PINS[i]);

    // Day raw vao cua so 3 mau
    pushHistory(i, raw);

    // Tinh gia tri da loc
    filteredReadings[i] = computeFilteredFromHistory(i);

    // Nghi ngan giua 2 cam bien de giam nguy co cross-talk
    delay(INTER_SENSOR_DELAY_MS);
  }
}

// ============================================================
// 12) MA HOA DU LIEU DE GUI UART
// ============================================================

uint16_t encodeDistanceX10(SensorReading s)
{
  if (!s.valid) {
    return 0;
  }

  float value = s.distanceCm * 10.0f;

  if (value < 0.0f) value = 0.0f;
  if (value > 65535.0f) value = 65535.0f;

  return (uint16_t)(value + 0.5f);
}

uint8_t buildValidMask()
{
  uint8_t mask = 0;

  if (filteredReadings[IDX_CL].valid) mask |= (1 << 0);
  if (filteredReadings[IDX_ML].valid) mask |= (1 << 1);
  if (filteredReadings[IDX_FC].valid) mask |= (1 << 2);
  if (filteredReadings[IDX_MR].valid) mask |= (1 << 3);
  if (filteredReadings[IDX_CR].valid) mask |= (1 << 4);

  return mask;
}

uint8_t calcChecksum(const SensorPacket& pkt)
{
  const uint8_t* p = (const uint8_t*)&pkt;
  uint8_t sum = 0;

  for (size_t i = 0; i < sizeof(SensorPacket) - 1; i++) {
    sum += p[i];
  }

  return sum;
}

SensorPacket buildPacket()
{
  SensorPacket pkt;

  pkt.magic       = 0xCAFE;
  pkt.version     = 1;
  pkt.seq         = packetSeq++;

  pkt.cornerLeft  = encodeDistanceX10(filteredReadings[IDX_CL]);
  pkt.midLeft     = encodeDistanceX10(filteredReadings[IDX_ML]);
  pkt.frontCenter = encodeDistanceX10(filteredReadings[IDX_FC]);
  pkt.midRight    = encodeDistanceX10(filteredReadings[IDX_MR]);
  pkt.cornerRight = encodeDistanceX10(filteredReadings[IDX_CR]);

  pkt.validMask   = buildValidMask();
  pkt.checksum    = 0;
  pkt.checksum    = calcChecksum(pkt);

  return pkt;
}

// ============================================================
// 13) DEBUG SERIAL
// ============================================================

void printOneSensor(const char* name, SensorReading s)
{
  Serial.print(name);
  Serial.print(": ");

  if (s.valid) {
    Serial.print(s.distanceCm, 1);
    Serial.print(" cm");
  } else {
    Serial.print("INVALID");
  }
}

void printAllFilteredSensors(uint8_t seq)
{
  Serial.print("SEQ=");
  Serial.print(seq);
  Serial.print(" | ");

  printOneSensor("CL", filteredReadings[IDX_CL]); Serial.print(" | ");
  printOneSensor("ML", filteredReadings[IDX_ML]); Serial.print(" | ");
  printOneSensor("FC", filteredReadings[IDX_FC]); Serial.print(" | ");
  printOneSensor("MR", filteredReadings[IDX_MR]); Serial.print(" | ");
  printOneSensor("CR", filteredReadings[IDX_CR]);

  Serial.println();
}

// ============================================================
// 14) SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(300);

  // Khai bao TRIG la OUTPUT
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
  }

  // Khai bao ECHO la INPUT
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(ECHO_PINS[i], INPUT);
  }

  // Khoi tao lich su median
  initHistory();

  // UART2: chi gui 1 chieu, RX pin co the de -1
  SensorLink.begin(SENSOR_UART_BAUD, SERIAL_8N1, -1, SENSOR_UART_TX_PIN);

  Serial.println();
  Serial.println("=== B2.1 SENSOR NODE START ===");
  Serial.println("Changes from B2:");
  Serial.println("- Faster loop");
  Serial.println("- median3 filter");
  Serial.println("- clamp max distance = 150 cm");
  Serial.println("- same UART packet format as B2");
}

// ============================================================
// 15) LOOP
// ============================================================

void loop()
{
  // 1) Doc 5 cam bien + loc
  readAllSensorsSequentiallyAndFilter();

  // 2) Tao packet
  SensorPacket pkt = buildPacket();

  // 3) Gui packet sang RX
  SensorLink.write((const uint8_t*)&pkt, sizeof(pkt));

  // 4) In log debug tren node sensor
  printAllFilteredSensors(pkt.seq);

  // 5) Delay cuoi vong nho hon ban B2 de cap nhat nhanh hon
  delay(LOOP_DELAY_MS);
}