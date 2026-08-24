#include "anna_link.h"

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "anna_state.h"

namespace {
HardwareSerial& kLinkPort = Serial1;
SemaphoreHandle_t g_tx_mutex = nullptr;

static inline void lockTx() {
  if (!g_tx_mutex) return;
  xSemaphoreTake(g_tx_mutex, portMAX_DELAY);
}

static inline void unlockTx() {
  if (!g_tx_mutex) return;
  xSemaphoreGive(g_tx_mutex);
}
} // namespace

void annaLinkInit() {
  // Master link: RX=44, TX=43 (ESP32-S3 Serial1 remap pins)
  kLinkPort.begin(1000000, SERIAL_8N1, 44, 43);
  if (!g_tx_mutex) g_tx_mutex = xSemaphoreCreateMutex();
}

void annaLinkPollRx(uint32_t now_ms, AnnaCommandCallback on_cmd, AnnaLedCallback on_led) {
  if (!on_cmd && !on_led) return;
  uint8_t type = 0;
  uint8_t buf[64];
  struct __attribute__((packed)) CommandFrameV1 {
    uint8_t axis_id, cmd;
    float   p0, p1;
  };
  static_assert(sizeof(CommandFrameV1) == 10, "CommandFrameV1 size unexpected");

  static bool warned_legacy = false;
  static bool warned_unknown = false;

  while (kLinkPort.available()) {
    uint16_t cap = sizeof(buf);
    if (!recvMessage(kLinkPort, type, buf, cap)) break;
    if (type == MSG_CMD) {
      // Mark link alive even if we can't parse the payload (e.g. mixed firmware versions).
      axisStateOnLinkRx(now_ms);

      if (cap == sizeof(CommandFrame)) {
        CommandFrame cmd{};
        memcpy(&cmd, buf, sizeof(cmd));
        if (on_cmd) on_cmd(cmd, now_ms);
      } else if (cap == sizeof(CommandFrameV1)) {
        // Backwards compatibility: older Beata firmware sent 2-float command frames.
        CommandFrameV1 old{};
        memcpy(&old, buf, sizeof(old));
        CommandFrame cmd{};
        cmd.axis_id = old.axis_id;
        cmd.cmd = old.cmd;
        cmd.p0 = old.p0;
        cmd.p1 = old.p1;
        cmd.p2 = 0.0f;
        if (on_cmd) on_cmd(cmd, now_ms);

        if (!warned_legacy) {
          warned_legacy = true;
          annaLinkSendEventText("link: legacy CMD frame (update Beata+Anna together for new commands)");
        }
      } else if (cap == sizeof(LedFrame)) {
        LedFrame frame{};
        memcpy(&frame, buf, sizeof(frame));
        if (frame.cmd == (uint8_t)CommandId::SetPixels) {
          if (on_led) on_led(frame, now_ms);
        } else if (!warned_unknown) {
          warned_unknown = true;
          char msg[64];
          snprintf(msg, sizeof(msg), "link: unknown CMD id=%u len=%u",
                   (unsigned)frame.cmd, (unsigned)cap);
          annaLinkSendEventText(msg);
        }
      } else {
        if (!warned_unknown) {
          warned_unknown = true;
          char msg[64];
          snprintf(msg, sizeof(msg), "link: unknown CMD len=%u (expected %u, %u or %u)",
                   (unsigned)cap,
                   (unsigned)sizeof(CommandFrame),
                   (unsigned)sizeof(CommandFrameV1),
                   (unsigned)sizeof(LedFrame));
          annaLinkSendEventText(msg);
        }
      }
    }
  }
}

void annaLinkSendStatus(const StatusFrame& st) {
  lockTx();
  sendStatus(kLinkPort, st);
  unlockTx();
}

void annaLinkSendCommand(const CommandFrame& cmd) {
  lockTx();
  sendCommand(kLinkPort, cmd);
  unlockTx();
}

void annaLinkSendEvent(const uint8_t* data, uint16_t len) {
  lockTx();
  sendEvent(kLinkPort, data, len);
  unlockTx();
}

void annaLinkSendEventText(const char* text) {
  if (!text) return;
  annaLinkSendEvent(reinterpret_cast<const uint8_t*>(text), (uint16_t)strlen(text));
}
