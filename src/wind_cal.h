#pragma once

// Auto-calibration for the ST60 sin/cos channels, replacing the fixed
// 2.0-7.0 V plausibility window (which rejected this unit's 1.93 V sine).
//
// Each channel's min and max are learned as the vane turns and kept in NVS.
// Until a channel has seen at least kMinLearnedSpan of swing, the default
// 1.5-6.5 V span is used. Each reading is normalised to about +-1:
//     n = (v - mid) / half_span
// and the pair is rejected only when a channel sits well outside its span, or
// when sin_n^2 + cos_n^2 is far from 1 (a broken wire or dead transducer).

#include <Arduino.h>
#include <Preferences.h>
#include <esp_log.h>
#include <math.h>

namespace windcal {

static constexpr float kDefaultLo = 1.5f, kDefaultHi = 6.5f;
static constexpr float kAbsMin = 0.5f, kAbsMax = 7.6f;  // never learn past
static constexpr float kMinLearnedSpan = 1.5f;
static constexpr float kMaxStep = 0.3f;  // learn only from steady readings
static constexpr float kOutsideFrac = 0.25f;
static constexpr float kMagMin = 0.5f, kMagMax = 1.5f;
static const char* kCalTag = "windcal";

struct Chan {
  float lo = NAN, hi = NAN;  // learned extremes, NAN until seen
  float prev = NAN;
  float n = NAN;             // latest normalised value
};
static Chan ch[2];  // 0 = sin (A1), 1 = cos (A2)
static bool dirty = false;
static Preferences prefs;

inline bool learned(int i) {
  return isfinite(ch[i].lo) && isfinite(ch[i].hi) &&
         ch[i].hi - ch[i].lo >= kMinLearnedSpan;
}
inline float lo(int i) { return learned(i) ? ch[i].lo : kDefaultLo; }
inline float hi(int i) { return learned(i) ? ch[i].hi : kDefaultHi; }

inline void load() {
  prefs.begin("windcal", true);
  ch[0].lo = prefs.getFloat("lo0", NAN);
  ch[0].hi = prefs.getFloat("hi0", NAN);
  ch[1].lo = prefs.getFloat("lo1", NAN);
  ch[1].hi = prefs.getFloat("hi1", NAN);
  prefs.end();
  ESP_LOGI(kCalTag, "sin %.2f-%.2f V%s, cos %.2f-%.2f V%s", lo(0), hi(0),
           learned(0) ? "" : " (default)", lo(1), hi(1),
           learned(1) ? "" : " (default)");
}

// Call every minute or so; writes NVS only when something changed.
inline void save() {
  if (!dirty) return;
  prefs.begin("windcal", false);
  prefs.putFloat("lo0", ch[0].lo);
  prefs.putFloat("hi0", ch[0].hi);
  prefs.putFloat("lo1", ch[1].lo);
  prefs.putFloat("hi1", ch[1].hi);
  prefs.end();
  dirty = false;
  ESP_LOGI(kCalTag, "saved: sin %.2f-%.2f V, cos %.2f-%.2f V", lo(0), hi(0),
           lo(1), hi(1));
}

// Forget the learned spans (back to defaults); spin the vane to relearn.
inline void reset() {
  for (auto& c : ch) c.lo = c.hi = NAN;
  dirty = true;
  save();
}

inline void learn(int i, float v) {
  Chan& c = ch[i];
  const bool steady = isfinite(c.prev) && fabsf(v - c.prev) < kMaxStep;
  c.prev = v;
  if (!steady || v < kAbsMin || v > kAbsMax) return;
  if (!isfinite(c.lo) || v < c.lo) {
    c.lo = v;
    dirty = true;
  }
  if (!isfinite(c.hi) || v > c.hi) {
    c.hi = v;
    dirty = true;
  }
}

// Volts in, normalised value out (NAN when the reading is not believable).
// Call for sin (i = 0) before cos (i = 1) in each cycle: the cos call also
// checks the pair's magnitude and blanks the angle if it is far from 1.
inline float normalise(int i, float v) {
  if (!isfinite(v)) return ch[i].n = NAN;
  learn(i, v);
  const float l = lo(i), h = hi(i);
  const float span = h - l;
  if (v < l - kOutsideFrac * span || v > h + kOutsideFrac * span) {
    return ch[i].n = NAN;
  }
  const float n = (v - (l + h) / 2.0f) / (span / 2.0f);
  ch[i].n = n;
  if (i == 1) {
    const float m = ch[0].n * ch[0].n + n * n;
    if (!isfinite(m) || m < kMagMin || m > kMagMax) return NAN;
  }
  return n;
}

inline bool both_learned() { return learned(0) && learned(1); }

}  // namespace windcal
