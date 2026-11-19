#pragma once

// NMEA 0183 receive on HALMET D2 (GPIO25), D2 pull and low-pass jumpers open.
// Wire the talker's A/+ (TX+) to D2 and its B/- (TX-) to the D2 ground
// terminal.
//
// Plain UART, 8N1. Starts at 4800 baud with the polarity read from the idle
// line (mark is high when A/B are the right way round). If bytes arrive but
// no sentence with a valid checksum turns up for 30 s, it moves on to the
// next of: 4800 direct, 4800 inverted, 38400 direct, 38400 inverted.
//
// Valid sentences are forwarded to the TCP clients on port 10110 alongside
// our own $WIMWV, and GPS time from RMC or ZDA feeds the NTP server.

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "gps_time_server.h"
#include "nmea0183_tcp.h"

namespace nmea0183rx {

static constexpr uart_port_t kPort = UART_NUM_2;
static constexpr int kRxPin = 25;  // HALMET D2
static constexpr uint32_t kRetestMs = 30000;
static constexpr size_t kMaxLine = 100;
static const char* kRxTag = "nmea0183-rx";

struct Mode {
  int baud;
  bool invert;
};
static const Mode kModes[] = {
    {4800, false}, {4800, true}, {38400, false}, {38400, true}};
static constexpr int kModeCount = sizeof(kModes) / sizeof(kModes[0]);

struct Stats {
  uint32_t bytes = 0, sentences = 0, bad_checksum = 0, time_fixes = 0;
};
static Stats stats;
static int mode = 0;
static bool ready = false;
static char line[kMaxLine + 1];
static size_t line_len = 0;
static uint32_t last_valid_ms = 0, retest_ms = 0, bytes_at_retest = 0;

inline const char* mode_name() {
  static char buf[24];
  snprintf(buf, sizeof(buf), "%d %s", kModes[mode].baud,
           kModes[mode].invert ? "inverted" : "direct");
  return buf;
}

inline void apply_mode() {
  uart_set_baudrate(kPort, kModes[mode].baud);
  uart_set_line_inverse(kPort, kModes[mode].invert ? UART_SIGNAL_RXD_INV
                                                   : UART_SIGNAL_INV_DISABLE);
  uart_flush_input(kPort);
  line_len = 0;
}

inline int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Field n (0 = the address field) of a sentence, without the checksum.
inline bool field(const char* s, int n, char* out, size_t out_len) {
  const char* p = s;
  for (int i = 0; i < n; i++) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
  }
  size_t k = 0;
  while (*p && *p != ',' && *p != '*' && k + 1 < out_len) out[k++] = *p++;
  out[k] = 0;
  return true;
}

// Days since 1970-01-01 for a civil date (Howard Hinnant's algorithm).
inline int64_t days_from_civil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

inline bool parse_hms(const char* t, double& secs) {
  if (strlen(t) < 6) return false;
  const int hh = (t[0] - '0') * 10 + (t[1] - '0');
  const int mm = (t[2] - '0') * 10 + (t[3] - '0');
  const double ss = atof(t + 4);
  if (hh > 23 || mm > 59 || ss >= 61.0) return false;
  secs = hh * 3600.0 + mm * 60.0 + ss;
  return true;
}

inline void offer_time(int y, int mo, int d, double secs, int64_t rx_us) {
  if (y < 2026 || mo < 1 || mo > 12 || d < 1 || d > 31) return;
  const int64_t utc_us =
      days_from_civil(y, mo, d) * 86400LL * 1000000LL + (int64_t)(secs * 1e6);
  halmet::GpsTimeServer::instance().offer_utc(utc_us, rx_us, "NMEA 0183");
  stats.time_fixes++;
}

// $--RMC,hhmmss.ss,A,...,ddmmyy,...   $--ZDA,hhmmss.ss,dd,mm,yyyy,...
inline void parse_time(const char* s, int64_t rx_us) {
  char f[16], t[16];
  double secs;
  if (strncmp(s + 3, "RMC", 3) == 0) {
    if (!field(s, 2, f, sizeof(f)) || strcmp(f, "A") != 0) return;
    if (!field(s, 1, t, sizeof(t)) || !parse_hms(t, secs)) return;
    if (!field(s, 9, f, sizeof(f)) || strlen(f) != 6) return;
    const int d = (f[0] - '0') * 10 + (f[1] - '0');
    const int mo = (f[2] - '0') * 10 + (f[3] - '0');
    const int y = 2000 + (f[4] - '0') * 10 + (f[5] - '0');
    offer_time(y, mo, d, secs, rx_us);
  } else if (strncmp(s + 3, "ZDA", 3) == 0) {
    char dd[4], mm[4], yy[6];
    if (!field(s, 1, t, sizeof(t)) || !parse_hms(t, secs)) return;
    if (!field(s, 2, dd, sizeof(dd)) || !field(s, 3, mm, sizeof(mm)) ||
        !field(s, 4, yy, sizeof(yy))) {
      return;
    }
    offer_time(atoi(yy), atoi(mm), atoi(dd), secs, rx_us);
  }
}

inline void on_line(int64_t rx_us) {
  line[line_len] = 0;
  if (line_len < 9 || (line[0] != '$' && line[0] != '!')) return;
  char* star = strrchr(line, '*');
  if (!star || star[1] == 0 || star[2] == 0) return;
  const int hi = hexval(star[1]), lo = hexval(star[2]);
  uint8_t cs = 0;
  for (char* p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
  if (hi < 0 || lo < 0 || cs != (uint8_t)(hi * 16 + lo)) {
    stats.bad_checksum++;
    return;
  }
  star[3] = 0;  // drop anything after the checksum
  stats.sentences++;
  last_valid_ms = millis();
  nmea0183::forward(line);
  if (line[0] == '$') parse_time(line + 1, rx_us);
}

inline void begin() {
  pinMode(kRxPin, INPUT);
  int high = 0;
  for (int i = 0; i < 100; i++) {
    high += digitalRead(kRxPin);
    delay(1);
  }
  mode = high < 50 ? 1 : 0;  // idle low: start inverted

  uart_config_t cfg = {};
  cfg.baud_rate = kModes[mode].baud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(kPort, 2048, 0, 0, nullptr, 0) != ESP_OK ||
      uart_param_config(kPort, &cfg) != ESP_OK ||
      uart_set_pin(kPort, UART_PIN_NO_CHANGE, kRxPin, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) != ESP_OK) {
    ESP_LOGE(kRxTag, "UART setup failed");
    return;
  }
  apply_mode();
  ready = true;
  last_valid_ms = retest_ms = millis();
  ESP_LOGI(kRxTag, "RX on GPIO%d (D2), idle %d/100 high, starting %s", kRxPin,
           high, mode_name());
}

// Call every 10 ms.
inline void poll() {
  if (!ready) return;
  uint8_t buf[128];
  int n;
  while ((n = uart_read_bytes(kPort, buf, sizeof(buf), 0)) > 0) {
    const int64_t rx_us = esp_timer_get_time();
    stats.bytes += n;
    for (int i = 0; i < n; i++) {
      const char c = (char)buf[i];
      if (c == '$' || c == '!') line_len = 0;  // a new sentence starts
      if (c == '\r' || c == '\n') {
        if (line_len > 0) on_line(rx_us);
        line_len = 0;
      } else if (line_len < kMaxLine) {
        line[line_len++] = c;
      } else {
        line_len = 0;  // overlong: not NMEA, drop it
      }
    }
  }

  const uint32_t ms = millis();
  if (ms - retest_ms >= kRetestMs) {
    if (stats.bytes != bytes_at_retest && ms - last_valid_ms >= kRetestMs) {
      mode = (mode + 1) % kModeCount;
      apply_mode();
      ESP_LOGW(kRxTag, "no valid sentences in 30 s, trying %s", mode_name());
    }
    retest_ms = ms;
    bytes_at_retest = stats.bytes;
  }
}

}  // namespace nmea0183rx
