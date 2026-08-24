#include "anna_state.h"

#include <Arduino.h>

#include "protocol_constants.h"

namespace {
struct AxisStateInternal {
  float    angle_deg_unwrapped = NAN;
  float    velocity_dps        = 0.0f;
  float    temp_c              = NAN;
  bool     temp_ok             = false;
  bool     encoder_ok          = false;
  float    range_mm            = NAN;
  bool     range_ok            = false;
  bool     limit_pressed       = false;
  uint8_t  fault_bits          = 0;
  bool     driver_requested_on = true;
  bool     teensy_seen         = false;
  uint32_t last_teensy_rx_ms   = 0;
};

static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;
static AxisStateInternal g_state;
} // namespace

void axisStateInit() {
  portENTER_CRITICAL(&g_state_mux);
  g_state = AxisStateInternal{};
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateOnLinkRx(uint32_t now_ms) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.teensy_seen = true;
  g_state.last_teensy_rx_ms = now_ms;
  portEXIT_CRITICAL(&g_state_mux);
}

bool axisStateLinkConnected(uint32_t now_ms) {
  portENTER_CRITICAL(&g_state_mux);
  bool seen = g_state.teensy_seen;
  uint32_t last = g_state.last_teensy_rx_ms;
  portEXIT_CRITICAL(&g_state_mux);
  return seen && ((int32_t)(now_ms - last) < (int32_t)kLinkDisconnectTimeoutMs);
}

void axisStateSetDriverRequested(bool on) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.driver_requested_on = on;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetLimitPressed(bool pressed) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.limit_pressed = pressed;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetFaultBits(uint8_t bits) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.fault_bits = bits;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetTemperature(float temp_c, bool ok) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.temp_ok = ok;
  g_state.temp_c = ok ? temp_c : NAN;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetEncoderOk(bool ok) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.encoder_ok = ok;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetKinematics(float angle_deg_unwrapped, float velocity_dps) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.angle_deg_unwrapped = angle_deg_unwrapped;
  g_state.velocity_dps = velocity_dps;
  portEXIT_CRITICAL(&g_state_mux);
}

void axisStateSetRange(float range_mm, bool ok) {
  portENTER_CRITICAL(&g_state_mux);
  g_state.range_ok = ok;
  g_state.range_mm = ok ? range_mm : NAN;
  portEXIT_CRITICAL(&g_state_mux);
}

AxisStateSnapshot axisStateRead(uint32_t now_ms) {
  AxisStateSnapshot out{};
  portENTER_CRITICAL(&g_state_mux);
  out.angle_deg_unwrapped = g_state.angle_deg_unwrapped;
  out.velocity_dps = g_state.velocity_dps;
  out.temp_c = g_state.temp_c;
  out.temp_ok = g_state.temp_ok;
  out.encoder_ok = g_state.encoder_ok;
  out.range_mm = g_state.range_mm;
  out.range_ok = g_state.range_ok;
  out.limit_pressed = g_state.limit_pressed;
  out.fault_bits = g_state.fault_bits;
  out.driver_requested_on = g_state.driver_requested_on;
  out.teensy_seen = g_state.teensy_seen;
  out.last_teensy_rx_ms = g_state.last_teensy_rx_ms;
  portEXIT_CRITICAL(&g_state_mux);
  out.link_connected = out.teensy_seen && ((int32_t)(now_ms - out.last_teensy_rx_ms) < (int32_t)kLinkDisconnectTimeoutMs);
  return out;
}
