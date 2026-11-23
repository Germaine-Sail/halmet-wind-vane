#pragma once

// Helm MOB button: supervised two-wire loop on A3, lamp on GPIO16.
//
// Loop (both resistors inside the button housing, A3 CCS jumper fitted):
//   A3+ -- 100R --+-- 150R --+-- A3-
//                 +-- NO ----+
// HALMET drives 10 mA round the loop, so A3 reads 10 mA x loop resistance:
// ~2.5 V normal, ~1.0 V pressed, high when the cable is cut, ~0 V shorted.
//
// Until A3 has shown one normal reading, ~0 V means no loop current at all
// (CCS jumper missing, or nothing wired) rather than a shorted cable, and is
// reported as "no-excitation".
//
// Lamp: GPIO16 -> 100R -> 2N7000 gate (100K gate to GND); the MOSFET switches
// the lamp's negative side. Steady on = healthy, 1 Hz flash = MOB active,
// fast flash = loop fault.
//
// MOB latches. Releasing the button never cancels it; to clear, release and
// then hold the button for 5 seconds.

#include <Arduino.h>
#include <esp_log.h>

#include <functional>

#include "halmet_analog.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/observablevalue.h"

namespace mob {

static constexpr int kLampPin = 16;
static constexpr int kAdsChannel = 2;  // A3
static constexpr unsigned int kSampleMs = 100;
static constexpr int kDebounceSamples = 2;  // 100-200 ms
static constexpr uint32_t kClearHoldMs = 5000;

// Terminal-voltage bands from the MOB handover. Calibrate against the real
// loop; anything between bands is treated as a fault.
static constexpr float kOpenAbove = 2.95f;  // the current source tops out ~3.0 V
static constexpr float kNormalMin = 2.0f, kNormalMax = 2.9f;
static constexpr float kDegradedMin = 1.4f;  // up to kNormalMin
static constexpr float kPressedMin = 0.6f;   // up to kDegradedMin
static constexpr float kShortBelow = 0.4f;

enum class Loop { kNormal, kDegraded, kPressed, kOpen, kShort, kNoExcitation,
                  kFault };

inline const char* loop_name(Loop l) {
  switch (l) {
    case Loop::kNormal: return "normal";
    case Loop::kDegraded: return "degraded";
    case Loop::kPressed: return "pressed";
    case Loop::kOpen: return "open";
    case Loop::kShort: return "short";
    case Loop::kNoExcitation: return "no-excitation (check CCS jumper/wiring)";
    default: return "fault";
  }
}

static bool seen_normal = false;

inline Loop classify(float v) {
  if (!isfinite(v)) return Loop::kFault;  // ADS did not answer
  if (v > kOpenAbove) return Loop::kOpen;
  if (v >= kNormalMin && v <= kNormalMax) return Loop::kNormal;
  if (v >= kDegradedMin && v < kNormalMin) return Loop::kDegraded;
  if (v >= kPressedMin && v < kDegradedMin) return Loop::kPressed;
  if (v < kShortBelow) return seen_normal ? Loop::kShort : Loop::kNoExcitation;
  return Loop::kFault;  // in a gap between bands
}

inline bool is_fault(Loop l) {
  return l == Loop::kOpen || l == Loop::kShort || l == Loop::kNoExcitation ||
         l == Loop::kFault;
}

static Loop stable = Loop::kNoExcitation;
static Loop candidate = Loop::kNoExcitation;
static int candidate_count = 0;
static bool active = false;
static bool released_since_trigger = false;
static uint32_t press_start_ms = 0;
static bool press_timing = false;
static bool lamp_on = false;
static float last_volts = NAN;
static std::function<void()> on_trigger;
static sensesp::ObservableValue<String>* mob_note = nullptr;
static sensesp::ObservableValue<String>* fault_note = nullptr;
static const char* kMobTag = "mob";

static const char* kNoteMob =
    "{\"state\":\"emergency\",\"method\":[\"visual\",\"sound\"],"
    "\"message\":\"Man overboard\"}";
static const char* kNoteMobClear =
    "{\"state\":\"normal\",\"method\":[],\"message\":\"MOB cleared\"}";

inline String fault_json(Loop l) {
  if (is_fault(l)) {
    return String("{\"state\":\"alarm\",\"method\":[\"visual\",\"sound\"],"
                  "\"message\":\"MOB button ") +
           loop_name(l) + "\"}";
  }
  if (l == Loop::kDegraded) {
    return "{\"state\":\"warn\",\"method\":[\"visual\"],"
           "\"message\":\"MOB button loop degraded - check for water\"}";
  }
  return "{\"state\":\"normal\",\"method\":[],\"message\":\"MOB button OK\"}";
}

inline void trigger() {
  active = true;
  released_since_trigger = false;
  press_timing = false;
  ESP_LOGW(kMobTag, "MAN OVERBOARD (A3 %.2f V)", last_volts);
  if (mob_note) mob_note->set(kNoteMob);
  if (on_trigger) on_trigger();
}

inline void clear() {
  active = false;
  press_timing = false;
  ESP_LOGW(kMobTag, "MOB cleared by long press");
  if (mob_note) mob_note->set(kNoteMobClear);
}

inline void on_sample(float v) {
  last_volts = v;
  Loop c = classify(v);
  if (c == candidate) {
    if (candidate_count < kDebounceSamples) candidate_count++;
  } else {
    candidate = c;
    candidate_count = 1;
  }
  if (candidate_count >= kDebounceSamples && c != stable) {
    ESP_LOGI(kMobTag, "button %s -> %s (%.2f V)", loop_name(stable),
             loop_name(c), v);
    stable = c;
    if (stable == Loop::kNormal) seen_normal = true;
    if (fault_note) fault_note->set(fault_json(stable));
    if (stable == Loop::kPressed && !active) trigger();
  }

  if (!active) return;
  if (stable != Loop::kPressed) {
    released_since_trigger = true;
    press_timing = false;
  } else if (released_since_trigger) {
    if (!press_timing) {
      press_timing = true;
      press_start_ms = millis();
    } else if (millis() - press_start_ms >= kClearHoldMs) {
      clear();
    }
  }
}

inline void lamp_tick() {
  static uint8_t phase = 0;
  phase = (phase + 1) & 7;  // 8 x 125 ms = 1 s
  if (is_fault(stable)) {
    lamp_on = phase & 1;  // 4 Hz
  } else if (active) {
    lamp_on = phase < 4;  // 1 Hz
  } else {
    lamp_on = true;  // steady
  }
  digitalWrite(kLampPin, lamp_on ? HIGH : LOW);
}

inline const char* mob_state() { return active ? "ACTIVE" : "off"; }
inline const char* button_state() { return loop_name(stable); }

inline void setup(Adafruit_ADS1115* ads, std::function<void()> trigger_cb) {
  on_trigger = trigger_cb;
  pinMode(kLampPin, OUTPUT);
  digitalWrite(kLampPin, LOW);

  auto* a3 = new halmet::ADS1115VoltageInput(ads, kAdsChannel, "/mob/a3",
                                             kSampleMs);
  a3->connect_to(new sensesp::LambdaConsumer<float>(
      [](float v) { on_sample(v); }));

  mob_note = new sensesp::ObservableValue<String>(kNoteMobClear);
  mob_note->connect_to(new sensesp::SKOutputRawJson("notifications.mob", ""));
  fault_note =
      new sensesp::ObservableValue<String>(fault_json(Loop::kNoExcitation));
  fault_note->connect_to(
      new sensesp::SKOutputRawJson("notifications.halmet.mobButton", ""));

  sensesp::event_loop()->onRepeat(125, []() { lamp_tick(); });
  // Re-send both notifications so a SignalK server that reconnects picks up
  // the current state.
  sensesp::event_loop()->onRepeat(10000, []() {
    mob_note->set(mob_note->get());
    fault_note->set(fault_note->get());
  });
  ESP_LOGI(kMobTag, "MOB button on A3 (CCS), lamp on GPIO%d", kLampPin);
}

}  // namespace mob
