/*
  ESPNOW_NOISE_NODE.ino
  ------------------------------------------------------------
  Dùng ESP32 thường để tạo traffic ESP-NOW hợp lệ trên 2.4GHz.
  Mục tiêu: stress test hệ RC ESP-NOW trong môi trường nhiều thiết bị.

  Không dùng để phá sóng công cộng.
  Chỉ dùng trong khu vực test riêng của bạn.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

// ============================================================
// CONFIG
// ============================================================

// Phải trùng với channel bạn muốn stress test.
// Test lần lượt 1 / 6 / 11.
static const uint8_t NOISE_CHANNEL = 6;

// Đổi ID khác nhau cho từng board.
static const uint8_t NOISE_NODE_ID = 1;

// 20ms = 50Hz, giống xe.
// 10ms = 100Hz, nặng hơn.
// 5ms  = 200Hz, rất nặng, chỉ dùng khi stress test.
static const uint32_t SEND_INTERVAL_MS = 10;

// Broadcast ESP-NOW
uint8_t BROADCAST_MAC[6] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

// ============================================================
// PACKET 32 BYTE
// ============================================================

#pragma pack(push, 1)
struct NoisePacket {
  uint32_t magic;
  uint8_t  nodeId;
  uint32_t seq;
  uint32_t ms;
  uint8_t  pad[19];
};
#pragma pack(pop)

static_assert(sizeof(NoisePacket) == 32,
              "NoisePacket must be 32 bytes");

// ============================================================
// STATS
// ============================================================

uint32_t seq = 0;

uint32_t sent = 0;
uint32_t ok = 0;
uint32_t fail = 0;
uint32_t callErr = 0;

uint32_t lastSendMs = 0;
uint32_t lastPrintMs = 0;

// ============================================================
// CALLBACK
// ============================================================

void handleSendStatus(esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) {
    ok++;
  } else {
    fail++;
  }
}

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

// ============================================================
// SEND
// ============================================================

void sendNoisePacket()
{
  NoisePacket pkt;

  pkt.magic = 0xDEADBEEF;
  pkt.nodeId = NOISE_NODE_ID;
  pkt.seq = seq++;
  pkt.ms = millis();

  for (size_t i = 0; i < sizeof(pkt.pad); i++) {
    pkt.pad[i] = (uint8_t)(esp_random() & 0xFF);
  }

  esp_err_t err =
      esp_now_send(BROADCAST_MAC,
                   (const uint8_t*)&pkt,
                   sizeof(pkt));

  sent++;

  if (err != ESP_OK) {
    callErr++;
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=== ESPNOW NOISE NODE ===");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  delay(100);

  if (esp_wifi_set_channel(NOISE_CHANNEL,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("ERROR: set channel failed");
    while (true) delay(1000);
  }

  Serial.print("Node ID         : ");
  Serial.println(NOISE_NODE_ID);

  Serial.print("STA MAC         : ");
  Serial.println(WiFi.macAddress());

  Serial.print("Noise channel   : ");
  Serial.println(NOISE_CHANNEL);

  Serial.print("Interval        : ");
  Serial.print(SEND_INTERVAL_MS);
  Serial.println(" ms");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init failed");
    while (true) delay(1000);
  }

  if (esp_now_register_send_cb(onDataSent) != ESP_OK) {
    Serial.println("ERROR: register send callback failed");
    while (true) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
  peerInfo.channel = NOISE_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  esp_err_t peerResult = esp_now_add_peer(&peerInfo);

  if (peerResult != ESP_OK &&
      peerResult != ESP_ERR_ESPNOW_EXIST) {
    Serial.println("ERROR: add broadcast peer failed");
    while (true) delay(1000);
  }

  lastSendMs = millis();
  lastPrintMs = millis();

  Serial.println("Noise node ready.");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastSendMs) >= SEND_INTERVAL_MS) {
    lastSendMs = nowMs;
    sendNoisePacket();
  }

  if ((uint32_t)(nowMs - lastPrintMs) >= 1000) {
    lastPrintMs = nowMs;

    Serial.print("NODE:");
    Serial.print(NOISE_NODE_ID);

    Serial.print(" ch:");
    Serial.print(NOISE_CHANNEL);

    Serial.print(" sent:");
    Serial.print(sent);

    Serial.print(" ok:");
    Serial.print(ok);

    Serial.print(" fail:");
    Serial.print(fail);

    Serial.print(" callErr:");
    Serial.println(callErr);
  }

  delay(0);
}