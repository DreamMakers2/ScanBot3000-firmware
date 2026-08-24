#pragma once

#include <stdint.h>

struct AxisStateSnapshot {
  float    angle_deg_unwrapped;
  float    velocity_dps;
  float    temp_c;
  bool     temp_ok;
  bool     encoder_ok;
  float    range_mm;
  bool     range_ok;
  bool     limit_pressed;
  uint8_t  fault_bits;
  bool     driver_requested_on;
  bool     teensy_seen;
  uint32_t last_teensy_rx_ms;
  bool     link_connected;
};

void axisStateInit();

void axisStateOnLinkRx(uint32_t now_ms);
bool axisStateLinkConnected(uint32_t now_ms);

void axisStateSetDriverRequested(bool on);
void axisStateSetLimitPressed(bool pressed);
void axisStateSetFaultBits(uint8_t bits);
void axisStateSetTemperature(float temp_c, bool ok);
void axisStateSetEncoderOk(bool ok);
void axisStateSetKinematics(float angle_deg_unwrapped, float velocity_dps);
void axisStateSetRange(float range_mm, bool ok);

AxisStateSnapshot axisStateRead(uint32_t now_ms);
