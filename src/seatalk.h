#pragma once

// SeaTalk 1 on HALMET.
//
// RX: SeaTalk data -> HALMET D2 (GPIO25), D2 pull and low-pass jumpers open.
//     Decoded in software from edge timestamps, not the UART: the UART can
//     only recover the 9th (command) bit through a parity-error flag that the
//     ESP-IDF driver does not tie to individual bytes. Timing the edges gives
//     all nine bits exactly, at either polarity.
// TX: GPIO17 -> 100R -> 2N7000 gate (100K gate to GND); the drain pulls the
//     SeaTalk data line to SeaTalk 0V. GPIO high = bus low, so the UART TX
//     signal is inverted. The 9th bit goes out as the UART parity bit, with
//     even/odd chosen per byte. Our own datagrams are read back on D2 to
//     confirm they went out clean.
//
// 4800 baud, 11-bit characters: start, 8 data (LSB first), command bit, stop.
// The command bit is set only on the first byte of each datagram; the low
// nibble of byte 1, plus 3, is the datagram's total length.

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <math.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

namespace seatalk {

static constexpr int kRxPin = 25;  // HALMET D2
static constexpr int kTxPin = 17;
static constexpr uart_port_t kTxPort = UART_NUM_2;
static constexpr int kBaud = 4800;
static constexpr float kBitUs = 1e6f / kBaud;  // 208.3 us
static constexpr uint32_t kIdleUs = 10000;     // quiet time before we talk
static constexpr uint32_t kDumpMs = 10000;     // hex dump after boot
static constexpr uint32_t kRetestMs = 30000;   // polarity re-test interval
static constexpr uint32_t kForeignHoldMs = 5000;
static const char* kStTag = "seatalk";

// ---------------------------------------------------------------- edge capture
struct Edge {
  uint32_t t;
  uint8_t level;
};
static constexpr uint16_t kEdgeBuf = 1024;
static Edge edge_buf[kEdgeBuf];
static volatile uint16_t edge_head = 0;
static volatile uint16_t edge_tail = 0;
static volatile uint32_t edge_overflow = 0;

static void IRAM_ATTR on_edge() {
  const uint32_t t = (uint32_t)esp_timer_get_time();
  const uint8_t l = (REG_READ(GPIO_IN_REG) >> kRxPin) & 1;
  const uint16_t next = (edge_head + 1) % kEdgeBuf;
  if (next == edge_tail) {
    edge_overflow = edge_overflow + 1;
    return;
  }
  edge_buf[edge_head].t = t;
  edge_buf[edge_head].level = l;
  edge_head = next;
}

// -------------------------------------------------------------------- state
struct Stats {
  uint32_t bytes = 0, datagrams = 0, framing = 0, incomplete = 0;
  uint32_t foreign_wind = 0;
  uint32_t tx_ok = 0, unverified = 0, collisions = 0, dropped = 0;
};
static Stats stats;

static bool invert = false;  // logical level = pin level XOR invert
static bool in_char = false;
static uint32_t t0 = 0;      // start-bit edge of the current character
static int bit_k = 0;        // next bit to sample, 1..10
static uint16_t bits = 0;
static uint8_t level = 1;    // current logical level
static uint32_t last_edge_us = 0;

// Pulse-width diagnostics: how long the line sits low / high between edges,
// in whole bit times (208 us at 4800 baud). Real SeaTalk shows almost all
// pulses at 1-10 bit times; lots of "short" pulses means noise or ringing,
// and widths clustering at odd fractions mean the wrong baud rate.
struct Widths {
  uint32_t low[12] = {0}, high[12] = {0};  // [0] = shorter than half a bit
  uint32_t min_low_us = UINT32_MAX, min_high_us = UINT32_MAX;
};
static Widths widths;
static constexpr uint32_t kDiagMs = 120000;  // log for 2 min after boot

static uint8_t dg[18];
static int dg_len = 0, dg_expect = 0;

static uint32_t boot_ms = 0;
static uint32_t last_valid_ms = 0;
static uint32_t retest_ms = 0;
static uint32_t bytes_at_retest = 0;
static uint32_t last_foreign_ms = 0;
static bool have_foreign = false;
static float foreign_awa_deg = NAN, foreign_aws_kn = NAN;

// Echo check for our own transmissions.
enum class Echo { kNone, kPending, kMatched, kMismatch };
static Echo echo = Echo::kNone;
static uint8_t echo_buf[18];
static int echo_len = 0;

static bool tx_ready = false;

// ----------------------------------------------------------------- decoding
inline void on_datagram(const uint8_t* d, int len) {
  stats.datagrams++;
  last_valid_ms = millis();

  if (echo == Echo::kPending) {
    echo = (len == echo_len && memcmp(d, echo_buf, len) == 0) ? Echo::kMatched
                                                              : Echo::kMismatch;
    if (echo == Echo::kMatched) return;  // our own, not foreign
  }

  if (d[0] == 0x10 && len == 4) {
    foreign_awa_deg = ((d[2] << 8) | d[3]) / 2.0f;
    stats.foreign_wind++;
    last_foreign_ms = millis();
    have_foreign = true;
  } else if (d[0] == 0x11 && len == 4) {
    float v = (d[2] & 0x7F) + (d[3] & 0x0F) / 10.0f;
    if (d[2] & 0x80) v *= 1.943844f;  // unit flag: m/s
    foreign_aws_kn = v;
    stats.foreign_wind++;
    last_foreign_ms = millis();
    have_foreign = true;
  }
}

inline void on_char(uint8_t b, bool cmd) {
  stats.bytes++;
  if (millis() - boot_ms < kDumpMs) {
    ESP_LOGI(kStTag, "rx %02X%s", b, cmd ? " CMD" : "");
  }
  if (cmd) {
    if (dg_len > 0) stats.incomplete++;
    dg[0] = b;
    dg_len = 1;
    dg_expect = 0;
    return;
  }
  if (dg_len == 0) return;  // stray data byte, no datagram open
  if (dg_len < (int)sizeof(dg)) dg[dg_len++] = b;
  if (dg_len == 2) dg_expect = (b & 0x0F) + 3;
  if (dg_expect > 0 && dg_len >= dg_expect) {
    on_datagram(dg, dg_expect);
    dg_len = 0;
    dg_expect = 0;
  }
}

inline uint32_t sample_time(int k) {
  return t0 + (uint32_t)((k + 0.5f) * kBitUs);
}

// Take the sample for bit_k at the current logical level.
inline void take_sample() {
  if (bit_k <= 9) {
    bits |= (uint16_t)level << (bit_k - 1);
    bit_k++;
    return;
  }
  in_char = false;  // bit 10 is the stop bit
  if (level == 1) {
    on_char(bits & 0xFF, (bits >> 8) & 1);
  } else {
    stats.framing++;
  }
}

inline void feed_edge(uint32_t t, uint8_t pin_level) {
  const uint8_t l = pin_level ^ (invert ? 1 : 0);
  if (last_edge_us != 0) {
    const uint32_t w = t - last_edge_us;  // how long `level` lasted
    int n = (int)(w / kBitUs + 0.5f);
    if (n > 11) n = 11;
    if (level == 0) {
      widths.low[n]++;
      if (w < widths.min_low_us) widths.min_low_us = w;
    } else if (n < 11) {  // long highs are just idle gaps
      widths.high[n]++;
      if (w < widths.min_high_us) widths.min_high_us = w;
    }
  }
  last_edge_us = t;
  while (in_char && (int32_t)(t - sample_time(bit_k)) > 0) take_sample();
  level = l;
  if (!in_char && l == 0) {
    in_char = true;
    t0 = t;
    bit_k = 1;
    bits = 0;
  }
}

inline void reset_decoder() {
  in_char = false;
  dg_len = dg_expect = 0;
  level = ((REG_READ(GPIO_IN_REG) >> kRxPin) & 1) ^ (invert ? 1 : 0);
}

// Drain captured edges and finish a character whose stop bit has passed.
inline void process() {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  while (edge_tail != edge_head) {
    Edge e = edge_buf[edge_tail];
    edge_tail = (edge_tail + 1) % kEdgeBuf;
    feed_edge(e.t, e.level);
  }
  while (in_char && (int32_t)(now - sample_time(bit_k)) > 0) take_sample();
}

// Called every millisecond from the event loop.
inline void poll() {
  process();

  // Polarity re-test: only if bytes keep arriving but none has formed a
  // valid datagram for 30 s. A quiet bus is not a reason to flip.
  const uint32_t ms = millis();
  if (ms - retest_ms >= kRetestMs) {
    const bool bytes_seen = stats.bytes != bytes_at_retest;
    const bool none_valid = ms - last_valid_ms >= kRetestMs;
    if (bytes_seen && none_valid) {
      invert = !invert;
      reset_decoder();
      ESP_LOGW(kStTag, "no valid datagrams in 30 s, RX polarity now %s",
               invert ? "inverted" : "direct");
    }
    retest_ms = ms;
    bytes_at_retest = stats.bytes;
  }
}

inline void log_widths() {
  if (millis() - boot_ms > kDiagMs) return;
  char lo[96], hi[96];
  int a = 0, b = 0;
  for (int i = 0; i < 12; i++) {
    a += snprintf(lo + a, sizeof(lo) - a, "%lu ", (unsigned long)widths.low[i]);
    b += snprintf(hi + b, sizeof(hi) - b, "%lu ", (unsigned long)widths.high[i]);
  }
  ESP_LOGI(kStTag, "widths <.5T,1T..10T,11T+  low: %s(min %lu us)", lo,
           (unsigned long)(widths.min_low_us == UINT32_MAX ? 0
                                                           : widths.min_low_us));
  ESP_LOGI(kStTag, "widths <.5T,1T..10T       high: %s(min %lu us)  "
           "overflow=%lu", hi,
           (unsigned long)(widths.min_high_us == UINT32_MAX
                               ? 0 : widths.min_high_us),
           (unsigned long)edge_overflow);
}

inline bool foreign_wind_recent() {
  return have_foreign && millis() - last_foreign_ms < kForeignHoldMs;
}

// -------------------------------------------------------------------- setup
inline void begin() {
  boot_ms = millis();
  pinMode(kRxPin, INPUT);

  // Polarity from the idle line: SeaTalk idles high (~12 V). Sample 100 ms;
  // mostly high on the pin means D2 passes the bus level straight through.
  int high = 0;
  for (int i = 0; i < 100; i++) {
    high += digitalRead(kRxPin);
    delay(1);
  }
  invert = high < 50;
  reset_decoder();
  last_valid_ms = retest_ms = millis();
  attachInterrupt(kRxPin, on_edge, CHANGE);
  ESP_LOGI(kStTag, "RX on GPIO%d, idle %d/100 high, polarity %s", kRxPin, high,
           invert ? "inverted" : "direct");

  uart_config_t cfg = {};
  cfg.baud_rate = kBaud;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_EVEN;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(kTxPort, 256, 0, 0, nullptr, 0) == ESP_OK &&
      uart_param_config(kTxPort, &cfg) == ESP_OK &&
      uart_set_pin(kTxPort, kTxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                   UART_PIN_NO_CHANGE) == ESP_OK) {
    uart_set_line_inverse(kTxPort, UART_SIGNAL_TXD_INV);
    tx_ready = true;
    ESP_LOGI(kStTag, "TX on GPIO%d (inverted for the 2N7000)", kTxPin);
  } else {
    ESP_LOGE(kStTag, "TX UART setup failed");
  }
}

// --------------------------------------------------------------------- send
// Returns true when the datagram went out and read back intact.
inline bool send(const uint8_t* d, size_t len) {
  if (!tx_ready || len == 0 || len > sizeof(echo_buf)) return false;
  process();
  const uint32_t now = (uint32_t)esp_timer_get_time();
  if (in_char || dg_len > 0 || level == 0 || now - last_edge_us < kIdleUs) {
    stats.dropped++;
    return false;
  }

  memcpy(echo_buf, d, len);
  echo_len = (int)len;
  echo = Echo::kPending;

  for (size_t i = 0; i < len; i++) {
    const uint8_t b = d[i];
    const int bit9 = (i == 0) ? 1 : 0;
    const int ones = __builtin_popcount(b) & 1;
    // Even parity sends parity bit = ones; odd sends the opposite.
    uart_set_parity(kTxPort, ones == bit9 ? UART_PARITY_EVEN : UART_PARITY_ODD);
    uart_write_bytes(kTxPort, (const char*)&b, 1);
    uart_wait_tx_done(kTxPort, pdMS_TO_TICKS(20));
  }
  delay(1);  // let the last stop bit pass the sample point
  process();

  bool ok = false;
  if (echo == Echo::kMatched) {
    stats.tx_ok++;
    ok = true;
  } else if (echo == Echo::kMismatch) {
    stats.collisions++;
  } else {
    stats.unverified++;  // nothing read back: check D2 wiring
  }
  echo = Echo::kNone;
  return ok;
}

// 0x10 apparent wind angle: 10 01 XX YY, XXYY/2 = degrees right of bow.
// Input is SignalK convention: -pi..+pi rad, positive to starboard.
inline bool send_awa(float rad) {
  if (!isfinite(rad)) return false;
  float deg = rad * 180.0f / (float)M_PI;
  if (deg < 0) deg += 360.0f;
  const uint16_t half = (uint16_t)lroundf(deg * 2.0f) % 720;
  const uint8_t d[4] = {0x10, 0x01, (uint8_t)(half >> 8),
                        (uint8_t)(half & 0xFF)};
  return send(d, sizeof(d));
}

// 0x11 apparent wind speed: 11 01 XX 0Y, (XX & 0x7F) + Y/10 knots.
inline bool send_aws(float mps) {
  if (!isfinite(mps) || mps < 0) return false;
  float kn = mps * 1.943844f;
  if (kn > 127.9f) kn = 127.9f;
  int whole = (int)kn;
  int tenth = (int)lroundf((kn - whole) * 10.0f);
  if (tenth >= 10) {
    whole++;
    tenth = 0;
  }
  const uint8_t d[4] = {0x11, 0x01, (uint8_t)(whole & 0x7F),
                        (uint8_t)(tenth & 0x0F)};
  return send(d, sizeof(d));
}

// 0x6E man overboard: 6E 07 00 00 00 00 00 00 00 00.
inline bool send_mob() {
  const uint8_t d[10] = {0x6E, 0x07, 0, 0, 0, 0, 0, 0, 0, 0};
  return send(d, sizeof(d));
}

}  // namespace seatalk
