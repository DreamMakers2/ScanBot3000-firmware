#pragma once

#include <stdint.h>

#include <TMC2209.h>

enum class DriverRequestType : uint8_t {
  SetRunCurrentPct,
  SetMicrosteps,
  SetStandstillMode,
  SetVelocitySps,
  SetVactual,
  MoveSteps,
  DumpSettings,
  DumpStatus,
};

struct DriverRequest {
  DriverRequestType type;
  float             f32 = 0.0f;
  float             f32b = 0.0f;
  int32_t           i32 = 0;
  uint16_t          u16 = 0;
  uint8_t           u8  = 0;
};

void annaDriverInit();
bool annaDriverOk();

bool annaDriverEnqueue(const DriverRequest& req);

void annaDriverSetDirectionInverted(bool inverted);

// Runs inside the 1 kHz control loop (single owner of driver UART transactions).
void annaDriverTick(uint32_t now_ms, bool link_ok, bool enable_requested, bool limit_pressed);

// Runs as often as possible; generates step/dir pulses when using AccelStepper.
// Returns true while motion is active (caller can sleep when false).
bool annaDriverServiceStepper();

uint16_t annaDriverMicrosteps();
uint8_t  annaDriverStandstillMode();

// Relative position in full steps since boot (or last limit press reset).
int32_t annaDriverRelativePositionSteps();

void annaDriverPersistStandstillMode(uint8_t mode);
