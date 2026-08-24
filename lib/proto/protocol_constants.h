#pragma once

#include <stddef.h>
#include <stdint.h>

// Protocol-level shared constants for both Beata (Teensy) and Anna (ESP32-S3).

// Command identifiers carried in CommandFrame::cmd. Numeric values must match on both ends.
enum class CommandId : uint8_t {
  Disable            = 0,
  Enable             = 1,
  SetCurrent         = 2,
  SetMicrosteps      = 3,
  DisplayHello       = 5,
  SetPixel           = 6,
  SetName            = 7,
  SetVactual         = 20,
  SetVelocitySteps   = 21,
  MoveStepsPerSecond = 22,
  SetStandstillMode  = 23,
  MoveSteps          = 24,
  TmcSettings        = 30,
  TmcStatus          = 31,
  Reboot             = 40,
  MeasureRange       = 60,
  SetPixels          = 61,
  StopAlert          = 90,
  Ping               = 250,
};

static constexpr size_t kLedFrameLedCount = 8;
static constexpr uint8_t kLedFrameFlagBrightness = 1u << 0;

// Consider the link disconnected if no valid RX frames arrive within this window.
static constexpr uint32_t kLinkDisconnectTimeoutMs = 500;

// Host ping cadence used to keep the link alive and to detect disconnects.
static constexpr uint32_t kLinkPingIntervalMs = 50;
