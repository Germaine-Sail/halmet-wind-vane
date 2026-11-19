#pragma once

// NMEA 0183 over TCP on port 10110 (the usual NMEA port), for plotters and
// apps on the boat Wi-Fi: our $WIMWV apparent wind plus every valid sentence
// received on D2. Up to four clients.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_log.h>
#include <math.h>

namespace nmea0183 {

static constexpr uint16_t kPort = 10110;
static constexpr int kMaxClients = 4;
static WiFiServer* server = nullptr;
static WiFiClient clients[kMaxClients];
static uint32_t sent = 0;       // our own sentences sent
static uint32_t forwarded = 0;  // sentences passed on from the D2 input

inline int client_count() {
  int n = 0;
  for (auto& c : clients) n += c.connected() ? 1 : 0;
  return n;
}

// Call every 100 ms. Starts the server once the network is up, then accepts
// new clients and drops dead ones.
inline void poll() {
  if (server == nullptr) {
    if (WiFi.getMode() == WIFI_OFF) return;
    server = new WiFiServer(kPort);
    server->begin();
    server->setNoDelay(true);
    ESP_LOGI("nmea0183", "TCP server on port %u", kPort);
  }
  WiFiClient incoming = server->accept();
  if (!incoming) return;
  for (auto& c : clients) {
    if (!c.connected()) {
      c.stop();
      c = incoming;
      return;
    }
  }
  incoming.stop();  // all slots busy
}

inline bool write_all(const char* text) {
  bool any = false;
  for (auto& c : clients) {
    if (c.connected()) {
      c.print(text);
      any = true;
    }
  }
  return any;
}

// Pass on a complete sentence received from elsewhere ("$...*hh").
inline void forward(const char* sentence) {
  char out[112];
  snprintf(out, sizeof(out), "%s\r\n", sentence);
  if (write_all(out)) forwarded++;
}

inline void send_line(const char* body) {
  uint8_t cs = 0;
  for (const char* p = body; *p; p++) cs ^= (uint8_t)*p;
  char line[96];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
  if (write_all(line)) sent++;
}

// $WIMWV,<angle 0-359.9>,R,<speed>,N,<A|V>
inline void send_mwv(float awa_rad, float aws_mps) {
  if (!isfinite(awa_rad) && !isfinite(aws_mps)) return;
  char ang[12] = "", spd[12] = "";
  if (isfinite(awa_rad)) {
    float deg = awa_rad * 180.0f / (float)M_PI;
    if (deg < 0) deg += 360.0f;
    snprintf(ang, sizeof(ang), "%.1f", deg);
  }
  if (isfinite(aws_mps)) snprintf(spd, sizeof(spd), "%.1f", aws_mps * 1.943844f);
  const bool valid = isfinite(awa_rad) && isfinite(aws_mps);
  char body[64];
  snprintf(body, sizeof(body), "WIMWV,%s,R,%s,N,%c", ang, spd,
           valid ? 'A' : 'V');
  send_line(body);
}

}  // namespace nmea0183
