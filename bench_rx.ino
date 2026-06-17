/*
  ESPNOW_BENCH_RX_50HZ.ino
  Đo độ ổn định ESP-NOW RX:
  - goodPackets
  - lostSeq
  - maxGapMs
  - gap > 50 / 100 / 200 / 300ms
  - RSSI nếu dùng Arduino-ESP32 core 3.x
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// ============================================================
// CONFIG
// ============================================================

// Test lần lượt: 1, 6, 11.
// TX và RX phải giống nhau.
static const uint8_t ESPNOW_CHANNEL = 6;

static const uint32_t PRINT_MS = 1000;
static const uint32_t LINK_TIMEOUT_MS = 300;

static const uint16_t PKT_MAGIC = 0xA55A;
static const uint8_t  PKT_VER   = 3;

// ============================================================
// PACKET 16 BYTE - giống ControlPacket của xe
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

struct RxItem {
  ControlPacket pkt;
  uint8_t srcMac[6];
  int8_t rssi;
  bool hasRssi;
};

QueueHandle_t rxQueue = nullptr;
static const uint8_t RX_QUEUE_LEN = 64;

// ============================================================
// STATS
// ============================================================

volatile uint32_t cbBadSize = 0;
volatile uint32_t cbQueueDrops = 0;

bool hasPacket = false;
ControlPacket latestPkt {};
uint8_t latestMac[6] = {0};

uint32_t goodPackets = 0;
uint32_t badMagic = 0;
uint32_t badVersion = 0;
uint32_t badChecksum = 0;

bool seqInit = false;
uint32_t currentSession = 0;
uint16_t lastSeq = 0;

uint32_t sessionChanges = 0;
uint32_t lostSeq = 0;
uint32_t duplicateSeq = 0;
uint32_t outOfOrderSeq = 0;

uint32_t lastGoodMs = 0;
uint32_t prevGoodMs = 0;

uint32_t maxGapMs = 0;
uint32_t gapOver50 = 0;
uint32_t gapOver100 = 0;
uint32_t gapOver200 = 0;
uint32_t gapOver300 = 0;

uint32_t gapSamples = 0;
uint32_t gapSumMs = 0;

int8_t lastRssi = 0;
int8_t minRssi = 127;
int32_t rssiSum = 0;
uint32_t rssiSamples = 0;

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

bool validatePacket(const ControlPacket& pkt)
{
  if (pkt.magic != PKT_MAGIC) {
    badMagic++;
    return false;
  }

  if (pkt.version != PKT_VER) {
    badVersion++;
    return false;
  }

  uint16_t expected =
      checksum16_xor((const uint8_t*)&pkt,
                     sizeof(ControlPacket) - sizeof(pkt.checksum));

  if (expected != pkt.checksum) {
    badChecksum++;
    return false;
  }

  return true;
}

void updateSeq(const ControlPacket& pkt)
{
  if (!seqInit || pkt.sessionId != currentSession) {
    seqInit = true;
    currentSession = pkt.sessionId;
    lastSeq = pkt.seq;
    sessionChanges++;
    prevGoodMs = 0;
    return;
  }

  uint16_t delta = (uint16_t)(pkt.seq - lastSeq);

  if (delta == 0) {
    duplicateSeq++;
    return;
  }

  if (delta < 0x8000) {
    if (delta > 1) {
      lostSeq += (uint32_t)(delta - 1);
    }

    lastSeq = pkt.seq;
    return;
  }

  outOfOrderSeq++;
}

void updateGap(uint32_t nowMs)
{
  if (prevGoodMs != 0) {
    uint32_t gap = nowMs - prevGoodMs;

    gapSamples++;
    gapSumMs += gap;

    if (gap > maxGapMs) maxGapMs = gap;
    if (gap > 50)  gapOver50++;
    if (gap > 100) gapOver100++;
    if (gap > 200) gapOver200++;
    if (gap > 300) gapOver300++;
  }

  prevGoodMs = nowMs;
}

void processItem(const RxItem& item)
{
  if (!validatePacket(item.pkt)) {
    return;
  }

  uint32_t nowMs = millis();

  updateSeq(item.pkt);
  updateGap(nowMs);

  if (item.hasRssi) {
    lastRssi = item.rssi;
    if (item.rssi < minRssi) minRssi = item.rssi;
    rssiSum += item.rssi;
    rssiSamples++;
  }

  latestPkt = item.pkt;
  memcpy(latestMac, item.srcMac, 6);

  hasPacket = true;
  lastGoodMs = nowMs;
  goodPackets++;
}

void enqueuePacket(const uint8_t* srcMac,
                   const uint8_t* data,
                   int len,
                   bool hasRssi,
                   int8_t rssi)
{
  if (len != (int)sizeof(ControlPacket)) {
    cbBadSize++;
    return;
  }

  if (rxQueue == nullptr) {
    cbQueueDrops++;
    return;
  }

  RxItem item;
  memcpy(&item.pkt, data, sizeof(ControlPacket));

  if (srcMac != nullptr) {
    memcpy(item.srcMac, srcMac, 6);
  } else {
    memset(item.srcMac, 0, 6);
  }

  item.hasRssi = hasRssi;
  item.rssi = rssi;

  if (xQueueSend(rxQueue, &item, 0) != pdPASS) {
    cbQueueDrops++;
  }
}

// Arduino-ESP32 core 3.x
#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onDataRecv(const esp_now_recv_info_t* info,
                const uint8_t* data,
                int len)
{
  const uint8_t* srcMac = nullptr;
  bool hasRssi = false;
  int8_t rssi = 0;

  if (info != nullptr) {
    srcMac = info->src_addr;

    if (info->rx_ctrl != nullptr) {
      rssi = info->rx_ctrl->rssi;
      hasRssi = true;
    }
  }

  enqueuePacket(srcMac, data, len, hasRssi, rssi);
}

#else

void onDataRecv(const uint8_t* mac,
                const uint8_t* data,
                int len)
{
  enqueuePacket(mac, data, len, false, 0);
}

#endif

bool linkOk(uint32_t nowMs)
{
  return hasPacket &&
         ((uint32_t)(nowMs - lastGoodMs) <= LINK_TIMEOUT_MS);
}

void printStatus()
{
  uint32_t nowMs = millis();

  uint32_t age = hasPacket ? nowMs - lastGoodMs : 999999UL;

  float avgGap = 0.0f;
  if (gapSamples > 0) {
    avgGap = (float)gapSumMs / (float)gapSamples;
  }

  float pdr = 0.0f;
  if ((goodPackets + lostSeq) > 0) {
    pdr = 100.0f * (float)goodPackets /
          (float)(goodPackets + lostSeq);
  }

  float avgRssi = 0.0f;
  if (rssiSamples > 0) {
    avgRssi = (float)rssiSum / (float)rssiSamples;
  }

  Serial.print("RX ch:");
  Serial.print(ESPNOW_CHANNEL);

  Serial.print(" link:");
  Serial.print(linkOk(nowMs) ? "OK" : "LOST");

  Serial.print(" age:");
  Serial.print(age);
  Serial.print("ms");

  Serial.print(" good:");
  Serial.print(goodPackets);

  Serial.print(" lost:");
  Serial.print(lostSeq);

  Serial.print(" dup:");
  Serial.print(duplicateSeq);

  Serial.print(" out:");
  Serial.print(outOfOrderSeq);

  Serial.print(" pdr:");
  Serial.print(pdr, 3);
  Serial.print("%");

  Serial.print(" maxGap:");
  Serial.print(maxGapMs);
  Serial.print("ms");

  Serial.print(" avgGap:");
  Serial.print(avgGap, 1);
  Serial.print("ms");

  Serial.print(" >50:");
  Serial.print(gapOver50);

  Serial.print(" >100:");
  Serial.print(gapOver100);

  Serial.print(" >200:");
  Serial.print(gapOver200);

  Serial.print(" >300:");
  Serial.print(gapOver300);

  Serial.print(" badCk:");
  Serial.print(badChecksum);

  Serial.print(" badSz:");
  Serial.print(cbBadSize);

  Serial.print(" qD:");
  Serial.print(cbQueueDrops);

  Serial.print(" rssi:");
  if (rssiSamples > 0) {
    Serial.print((int)lastRssi);
    Serial.print(" avg:");
    Serial.print(avgRssi, 1);
    Serial.print(" min:");
    Serial.print((int)minRssi);
  } else {
    Serial.print("NA");
  }

  Serial.print(" src:");
  printMac(latestMac);

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
  Serial.println("=== ESPNOW BENCH RX 50HZ ===");

  rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxItem));

  if (rxQueue == nullptr) {
    Serial.println("ERROR: cannot create rxQueue");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  delay(100);

  if (esp_wifi_set_channel(ESPNOW_CHANNEL,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("ERROR: set channel failed");
    while (true) delay(1000);
  }

  Serial.print("RX STA MAC      : ");
  Serial.println(WiFi.macAddress());

  Serial.print("ESP-NOW channel : ");
  Serial.println(ESPNOW_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init failed");
    while (true) delay(1000);
  }

  if (esp_now_register_recv_cb(onDataRecv) != ESP_OK) {
    Serial.println("ERROR: register recv callback failed");
    while (true) delay(1000);
  }

  lastPrintMs = millis();

  Serial.println("RX ready. Copy RX STA MAC into TX RX_MAC[].");
}

void loop()
{
  RxItem item;

  while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
    processItem(item);
  }

  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastPrintMs) >= PRINT_MS) {
    lastPrintMs = nowMs;
    printStatus();
  }

  delay(0);
}