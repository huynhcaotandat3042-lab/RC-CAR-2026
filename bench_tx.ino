/*
  ESPNOW_BENCH_TX_50HZ.ino
  Gửi packet 16 byte ở 50Hz để benchmark ESP-NOW.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <esp_system.h>

// ============================================================
// CONFIG
// ============================================================

// Dán RX STA MAC vào đây.
uint8_t RX_MAC[6] = { 0xB0, 0xCB, 0xD8, 0x8B, 0x6E, 0x14 };

// Test lần lượt: 1, 6, 11.
// TX và RX phải giống nhau.
static const uint8_t ESPNOW_CHANNEL = 6;

static const uint32_t TX_UPDATE_MS = 20;   // 50Hz
static const uint32_t PRINT_MS = 1000;

static const uint16_t PKT_MAGIC = 0xA55A;
static const uint8_t  PKT_VER   = 3;

static const uint8_t MODE_MANUAL = 0;

// ============================================================
// PACKET 16 BYTE
// ============================================================

#pragma pack(push, 1)
struct ControlPacket {
  uint16_t magic;
  uint8_t  version;
  uint32_t sessionId;
  uint16_t seq;
  int16_t  steerUs;
  int16_t  throttleUs;
  uint8_t  mode;
  uint16_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) == 16,
              "ControlPacket must be 16 bytes");

// ============================================================
// STATS
// ============================================================

uint32_t sessionId = 0;
uint16_t seq = 0;

uint32_t sent = 0;
uint32_t sendOk = 0;
uint32_t sendFail = 0;
uint32_t sendCallErr = 0;

uint32_t lastSendMs = 0;
uint32_t lastCbMs = 0;
uint32_t lastPrintMs = 0;

// ============================================================
// HELPERS
// ============================================================

uint16_t checksum16_xor(const uint8_t* data, size_t len)
{
  uint16_t x = 0;

  for (size_t i = 0; i < len; i++) {
    x ^= data[i];
    x = (uint16_t)((x << 1) | (x >> 15));
  }

  return x;
}

void printMac(const uint8_t mac[6])
{
  Serial.printf(
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
}

void handleSendStatus(esp_now_send_status_t status)
{
  lastCbMs = millis();

  if (status == ESP_NOW_SEND_SUCCESS) {
    sendOk++;
  } else {
    sendFail++;
  }
}

// Arduino-ESP32 core 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onDataSent(const wifi_tx_info_t* info,
                esp_now_send_status_t status)
{
  (void)info;
  handleSendStatus(status);
}

#else

void onDataSent(const uint8_t* mac,
                esp_now_send_status_t status)
{
  (void)mac;
  handleSendStatus(status);
}

#endif

void makePacket(ControlPacket& pkt)
{
  pkt.magic = PKT_MAGIC;
  pkt.version = PKT_VER;
  pkt.sessionId = sessionId;
  pkt.seq = seq++;

  // Giá trị giả, mục đích chỉ là giữ packet giống xe thật.
  pkt.steerUs = 1500;
  pkt.throttleUs = 1500;
  pkt.mode = MODE_MANUAL;

  pkt.checksum = 0;
  pkt.checksum =
      checksum16_xor((const uint8_t*)&pkt,
                     sizeof(ControlPacket) - sizeof(pkt.checksum));
}

void sendPacket()
{
  ControlPacket pkt;
  makePacket(pkt);

  esp_err_t err =
      esp_now_send(RX_MAC,
                   (const uint8_t*)&pkt,
                   sizeof(pkt));

  sent++;
  lastSendMs = millis();

  if (err != ESP_OK) {
    sendCallErr++;
  }
}

void printStatus()
{
  uint32_t nowMs = millis();

  float failRate = 0.0f;
  if ((sendOk + sendFail) > 0) {
    failRate = 100.0f * (float)sendFail /
               (float)(sendOk + sendFail);
  }

  uint32_t cbAge =
      lastCbMs == 0 ? 999999UL : nowMs - lastCbMs;

  Serial.print("TX ch:");
  Serial.print(ESPNOW_CHANNEL);

  Serial.print(" seq:");
  Serial.print(seq);

  Serial.print(" sent:");
  Serial.print(sent);

  Serial.print(" ok:");
  Serial.print(sendOk);

  Serial.print(" fail:");
  Serial.print(sendFail);

  Serial.print(" callErr:");
  Serial.print(sendCallErr);

  Serial.print(" fail%:");
  Serial.print(failRate, 3);

  Serial.print(" cbAge:");
  Serial.print(cbAge);
  Serial.print("ms");

  Serial.print(" peer:");
  printMac(RX_MAC);

  Serial.println();
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESPNOW BENCH TX 50HZ ===");

  sessionId = esp_random();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  delay(100);

  if (esp_wifi_set_channel(ESPNOW_CHANNEL,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("ERROR: set channel failed");
    while (true) delay(1000);
  }

  Serial.print("TX STA MAC      : ");
  Serial.println(WiFi.macAddress());

  Serial.print("RX peer MAC     : ");
  printMac(RX_MAC);
  Serial.println();

  Serial.print("ESP-NOW channel : ");
  Serial.println(ESPNOW_CHANNEL);

  Serial.print("sessionId       : 0x");
  Serial.println(sessionId, HEX);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init failed");
    while (true) delay(1000);
  }

  if (esp_now_register_send_cb(onDataSent) != ESP_OK) {
    Serial.println("ERROR: register send callback failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RX_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: esp_now_add_peer failed");
    while (true) delay(1000);
  }

  lastSendMs = millis();
  lastPrintMs = millis();

  Serial.println("TX ready.");
}

void loop()
{
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastSendMs) >= TX_UPDATE_MS) {
    sendPacket();
  }

  if ((uint32_t)(nowMs - lastPrintMs) >= PRINT_MS) {
    lastPrintMs = nowMs;
    printStatus();
  }

  delay(0);
}