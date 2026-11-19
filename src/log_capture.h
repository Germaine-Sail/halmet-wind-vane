#pragma once

// Keeps the most recent ~32 KB of the serial log in RAM so it can be
// downloaded from the web UI (/api/log, linked from the /update page) without
// a USB cable. Everything still goes to the serial port as before.

#include <Arduino.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdarg>

namespace logcap {

static constexpr size_t kRingSize = 32 * 1024;
static char* ring = nullptr;
static size_t head = 0;      // next write position
static bool wrapped = false;
static SemaphoreHandle_t lock = nullptr;
static vprintf_like_t prev_vprintf = nullptr;
static char line_buf[256];   // guarded by `lock`

static int capture_vprintf(const char* fmt, va_list args) {
  va_list copy;
  va_copy(copy, args);
  // Never block a logging task for long: if the ring is busy (e.g. a
  // download in progress), this line just goes to serial only.
  if (ring && lock && xSemaphoreTake(lock, pdMS_TO_TICKS(5)) == pdTRUE) {
    int n = vsnprintf(line_buf, sizeof(line_buf), fmt, copy);
    if (n > 0) {
      if ((size_t)n >= sizeof(line_buf)) n = sizeof(line_buf) - 1;
      for (int i = 0; i < n; i++) {
        ring[head++] = line_buf[i];
        if (head == kRingSize) {
          head = 0;
          wrapped = true;
        }
      }
    }
    xSemaphoreGive(lock);
  }
  va_end(copy);
  return prev_vprintf ? prev_vprintf(fmt, args) : vprintf(fmt, args);
}

// Call as early as possible in setup(), right after SetupLogging().
inline void begin() {
  ring = (char*)malloc(kRingSize);
  lock = xSemaphoreCreateMutex();
  if (!ring || !lock) return;
  prev_vprintf = esp_log_set_vprintf(capture_vprintf);
}

// GET /api/log: the captured log as a text file, oldest line first.
static esp_err_t handle_log_get(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=\"halmet-wind-log.txt\"");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (!ring || !lock || xSemaphoreTake(lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
    httpd_resp_sendstr(req, "log capture unavailable\n");
    return ESP_OK;
  }
  // Sent straight from the ring while holding the lock, so no 32 KB copy is
  // needed; lines logged meanwhile go to serial only (see capture_vprintf).
  if (wrapped) httpd_resp_send_chunk(req, ring + head, kRingSize - head);
  httpd_resp_send_chunk(req, ring, head);
  xSemaphoreGive(lock);
  httpd_resp_send_chunk(req, nullptr, 0);
  return ESP_OK;
}

}  // namespace logcap
