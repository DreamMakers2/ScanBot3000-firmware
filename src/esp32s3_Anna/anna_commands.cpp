#include "anna_commands.h"

#include <Arduino.h>
#include <string.h>

#include "anna_driver.h"
#include "anna_link.h"
#include "anna_pins.h"
#include "anna_state.h"
#include "anna_tasks.h"
#include "anna_ui.h"

namespace {
static void handleEnableDisable(const CommandFrame& cmd, bool on) {
  (void)cmd;
  axisStateSetDriverRequested(on);
}

static void handleSetCurrent(const CommandFrame& cmd) {
  DriverRequest req{};
  req.type = DriverRequestType::SetRunCurrentPct;
  req.i32 = (int32_t)lrintf(cmd.p0);
  (void)annaDriverEnqueue(req);
}

static void handleSetMicrosteps(const CommandFrame& cmd) {
  DriverRequest req{};
  req.type = DriverRequestType::SetMicrosteps;
  long ms = lrintf(cmd.p0);
  if (ms < 1) ms = 1;
  if (ms > 256) ms = 256;
  req.u16 = (uint16_t)ms;
  (void)annaDriverEnqueue(req);
}

static void handleHello(const CommandFrame&) { annaUiShowMessage("Hello", 2000); }

static void handleSetPixel(const CommandFrame& cmd) {
  int idx = (int)lrintf(cmd.p0);
  if (idx < 0) idx = 0;
  if (idx >= (int)kNeoPixelCount) idx = (int)kNeoPixelCount - 1;
  uint32_t rgb = (uint32_t)cmd.p1;
  uint8_t bri = cmd.axis_id;
  annaUiSetPixelOverride((uint8_t)idx, rgb, bri);
}

static void handleSetPixelsFrame(const LedFrame& frame) {
  if (frame.cmd != (uint8_t)CommandId::SetPixels) return;
  annaUiSetLedFrame(frame);
}

static void handleSetName(const CommandFrame& cmd) {
  uint32_t chunk0 = 0;
  uint32_t chunk1 = 0;
  memcpy(&chunk0, &cmd.p0, sizeof(chunk0));
  memcpy(&chunk1, &cmd.p1, sizeof(chunk1));
  char name_buf[9] = {};
  memcpy(name_buf, &chunk0, sizeof(chunk0));
  memcpy(name_buf + 4, &chunk1, sizeof(chunk1));
  size_t len = cmd.axis_id;
  if (len > 8) len = 8;
  name_buf[len] = '\0';
  annaUiSetDeviceName(name_buf, len);
  const bool is_z_axis = (len == 1 && (name_buf[0] == 'Z' || name_buf[0] == 'z'));
  annaDriverSetDirectionInverted(is_z_axis);
  const bool is_r_axis = (len == 1 && (name_buf[0] == 'R' || name_buf[0] == 'r'));
  if (is_r_axis) {
    annaTasksConfigureAxisR();
    annaUiConfigureAxisR();
  } else {
    annaTasksConfigureAxisNonR();
  }
}

static void handleStandstillMode(const CommandFrame& cmd) {
  uint8_t requested = (uint8_t)TMC2209::NORMAL;
  if (isfinite(cmd.p0)) {
    long val = lrintf(cmd.p0);
    if (val < 0) val = 0;
    if (val > 255) val = 255;
    requested = (uint8_t)val;
  }
  annaDriverPersistStandstillMode(requested);
  DriverRequest req{};
  req.type = DriverRequestType::SetStandstillMode;
  req.u8 = requested;
  (void)annaDriverEnqueue(req);
}

static void handleVelocitySps(const CommandFrame& cmd) {
  DriverRequest req{};
  req.type = DriverRequestType::SetVelocitySps;
  req.f32 = cmd.p0;
  (void)annaDriverEnqueue(req);
}

static void handleMoveSteps(const CommandFrame& cmd) {
  DriverRequest req{};
  req.type = DriverRequestType::MoveSteps;
  if (isfinite(cmd.p0)) {
    req.i32 = (int32_t)lrintf(cmd.p0);
  } else {
    req.i32 = 0;
  }
  req.f32 = cmd.p1;  // velocity (steps/s)
  req.f32b = cmd.p2; // acceleration (steps/s^2)
  (void)annaDriverEnqueue(req);
}

static void handleVactual(const CommandFrame& cmd) {
  DriverRequest req{};
  req.type = DriverRequestType::SetVactual;
  req.i32 = (int32_t)lrintf(cmd.p0);
  (void)annaDriverEnqueue(req);
}

static void handleTmcSettings(const CommandFrame&) {
  DriverRequest req{};
  req.type = DriverRequestType::DumpSettings;
  (void)annaDriverEnqueue(req);
}

static void handleTmcStatus(const CommandFrame&) {
  DriverRequest req{};
  req.type = DriverRequestType::DumpStatus;
  (void)annaDriverEnqueue(req);
}

static void handleReboot(const CommandFrame&) {
  annaLinkSendEventText("reboot.request");
  delay(20);
  ESP.restart();
}

static void handleStopAlert(const CommandFrame&) { annaUiTriggerStopAlert(); }

static void handleMeasureRange(const CommandFrame& cmd) {
  float duration_s = cmd.p0;
  if (!isfinite(duration_s) || duration_s <= 0.0f) {
    annaLinkSendEventText("measure.reject: duration invalid");
    return;
  }
  if (!annaUiRequestMeasure(duration_s)) {
    annaLinkSendEventText("measure.reject: axis not R");
  }
}

using HandlerFn = void (*)(const CommandFrame& cmd);
struct DispatchEntry {
  CommandId id;
  HandlerFn fn;
};

static const DispatchEntry kDispatch[] = {
  { CommandId::Disable,            [](const CommandFrame& c){ handleEnableDisable(c, false); } },
  { CommandId::Enable,             [](const CommandFrame& c){ handleEnableDisable(c, true); } },
  { CommandId::SetCurrent,         handleSetCurrent },
  { CommandId::SetMicrosteps,      handleSetMicrosteps },
  { CommandId::DisplayHello,       handleHello },
  { CommandId::SetPixel,           handleSetPixel },
  { CommandId::SetName,            handleSetName },
  { CommandId::SetStandstillMode,  handleStandstillMode },
  { CommandId::SetVelocitySteps,   handleVelocitySps },
  { CommandId::MoveStepsPerSecond, handleVelocitySps },
  { CommandId::MoveSteps,          handleMoveSteps },
  { CommandId::SetVactual,         handleVactual },
  { CommandId::TmcSettings,        handleTmcSettings },
  { CommandId::TmcStatus,          handleTmcStatus },
  { CommandId::Reboot,             handleReboot },
  { CommandId::StopAlert,          handleStopAlert },
  { CommandId::MeasureRange,       handleMeasureRange },
};

} // namespace

void annaHandleCommand(const CommandFrame& cmd, uint32_t now_ms) {
  axisStateOnLinkRx(now_ms);

  const CommandId id = static_cast<CommandId>(cmd.cmd);
  for (const auto& entry : kDispatch) {
    if (entry.id == id) {
      entry.fn(cmd);
      return;
    }
  }
}

void annaHandleLedFrame(const LedFrame& frame, uint32_t now_ms) {
  axisStateOnLinkRx(now_ms);
  handleSetPixelsFrame(frame);
}
