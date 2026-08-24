#include "anna_tasks.h"

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>

#include <AS5600.h>
#include <Bounce2.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "anna_commands.h"
#include "anna_driver.h"
#include "anna_link.h"
#include "anna_pins.h"
#include "anna_state.h"
#include "anna_ui.h"

namespace {
// Encoder bus is separate from the shared OLED bus.
TwoWire g_wire_encoder(1);
AS5600 g_as5600(&g_wire_encoder);
bool g_as_ok = false;
volatile bool g_encoder_enabled = false;
bool g_axis_is_r = false;
bool g_encoder_configured = false;

OneWire g_onewire(PIN_ONEWIRE);
DallasTemperature g_ds18b20(&g_onewire);
bool g_temp_ok = false;

Bounce g_limit_btn;
Bounce g_home_btn;

static constexpr BaseType_t kCoreHeavy = 0;
static constexpr BaseType_t kCoreStepper = 1;

TaskHandle_t g_h_ctl = nullptr;
TaskHandle_t g_h_sns = nullptr;
TaskHandle_t g_h_ui  = nullptr;
TaskHandle_t g_h_tmp = nullptr;
TaskHandle_t g_h_tmp_retry = nullptr;
TaskHandle_t g_h_link = nullptr;
TaskHandle_t g_h_stp = nullptr;

// AS5600 unwrap/velocity pipeline state.
uint16_t g_last_raw = 0;
int32_t  g_turns = 0;
uint32_t g_last_vel_us = 0;
float    g_last_pos_deg = NAN;
uint16_t g_as_error_streak = 0;
uint32_t g_as_last_recover_ms = 0;

// Local homing button (GPIO1 on the axis controller). We forward a request to the host (Teensy),
// which runs the full multi-axis homing sequence.
static constexpr uint32_t kHomeDebounceMs = 10;

static void homeSendEvent(const char* text) {
  if (!text) return;
  annaLinkSendEventText(text);
}

static void homeTick(bool link_ok, bool limit_pressed, bool home_btn_fell) {
  if (!home_btn_fell) return;

  if (!link_ok) {
    annaUiShowMessage("NO LINK", 1200);
    homeSendEvent("home.reject: link not connected");
    return;
  }
  if (!limit_pressed) {
    annaUiShowMessage("SW OFF", 1200);
    homeSendEvent("home.reject: switch not pressed");
    return;
  }

  homeSendEvent("home.request");
  annaUiShowMessage("HOME REQ", 800);
}

static inline void configureAs5600ForBandwidth() {
  (void)g_as5600.setPowerMode(AS5600_POWERMODE_NOMINAL);
  (void)g_as5600.setSlowFilter(AS5600_SLOW_FILT_2X);
  (void)g_as5600.setFastFilter(AS5600_FAST_FILT_LSB6);
  (void)g_as5600.setHysteresis(AS5600_HYST_LSB1);
  (void)g_as5600.setWatchDog(AS5600_WATCHDOG_OFF);
}

static bool recoverAs5600(uint32_t now_ms) {
  if (!g_encoder_enabled) return false;
  static constexpr uint32_t kMinRecoverIntervalMs = 250;
  if ((int32_t)(now_ms - g_as_last_recover_ms) < (int32_t)kMinRecoverIntervalMs) return false;
  g_as_last_recover_ms = now_ms;

  // Re-init the encoder I2C bus and re-probe the device. This helps if the bus
  // glitches under stepper EMI and the library starts returning cached angles.
  g_wire_encoder.end();
  delay(2);
  g_wire_encoder.begin(I2C_ENCODER_SDA_PIN, I2C_ENCODER_SCL_PIN, I2C_ENCODER_CLOCK_HZ);
  g_wire_encoder.setClock(I2C_ENCODER_CLOCK_HZ);
  g_wire_encoder.setTimeOut(50);

  g_as_ok = g_as5600.begin();
  axisStateSetEncoderOk(g_as_ok);
  if (!g_as_ok) return false;

  configureAs5600ForBandwidth();

  // Re-seed unwrap pipeline to avoid a massive delta after recovery.
  uint16_t raw = g_as5600.readAngle();
  g_last_raw = raw;
  g_last_pos_deg = (g_turns * 360.0f) + raw * (360.0f / 4096.0f);
  g_last_vel_us = micros();
  axisStateSetKinematics(g_last_pos_deg, 0.0f);

  return true;
}

static inline void processAs5600Raw(uint16_t raw) {
  int delta = (int)raw - (int)g_last_raw;
  if (delta > 2048)      { g_turns--; }
  else if (delta < -2048){ g_turns++; }
  g_last_raw = raw;

  float angle_abs = raw * (360.0f / 4096.0f);
  float angle_unwrapped = (g_turns * 360.0f) + angle_abs;

  uint32_t now_us = micros();
  float vel = 0.0f;
  float dt = (now_us - g_last_vel_us) / 1e6f;
  if (dt > 0.0f && isfinite(g_last_pos_deg)) {
    vel = (angle_unwrapped - g_last_pos_deg) / dt;
  }
  g_last_pos_deg = angle_unwrapped;
  g_last_vel_us = now_us;

  axisStateSetKinematics(angle_unwrapped, vel);
}

static void controlTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();

  bool last_limit = false;
  for (;;) {
    vTaskDelayUntil(&last, period);

    g_limit_btn.update();
    g_home_btn.update();
    bool limit_pressed = !g_limit_btn.read(); // active-low
    bool home_btn_fell = g_home_btn.fell();   // active-low
    if (limit_pressed != last_limit) {
      annaUiNotify();
      last_limit = limit_pressed;
    }
    axisStateSetLimitPressed(limit_pressed);

    uint32_t now_ms = millis();
    bool link_ok = axisStateLinkConnected(now_ms);
    if (!link_ok) {
      axisStateSetDriverRequested(false);
    }

    homeTick(link_ok, limit_pressed, home_btn_fell);

    AxisStateSnapshot s = axisStateRead(now_ms);
    annaDriverTick(now_ms, link_ok, s.driver_requested_on, limit_pressed);
  }
}

static void linkTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, period);
    uint32_t now_ms = millis();
    uint32_t now_us = micros();

    annaLinkPollRx(now_ms, annaHandleCommand, annaHandleLedFrame);

    AxisStateSnapshot s = axisStateRead(now_ms);
    StatusFrame st{};
    st.ts_us     = now_us;
    st.pos_steps = annaDriverRelativePositionSteps(); // RP: full steps since boot / last limit hit
    st.angle_deg = (s.encoder_ok && isfinite(s.angle_deg_unwrapped)) ? s.angle_deg_unwrapped : 0.0f;
    st.dist_mm   = (s.range_ok && isfinite(s.range_mm)) ? s.range_mm : -1.0f;
    st.temp_c    = (s.temp_ok && isfinite(s.temp_c)) ? s.temp_c : NAN;
    st.limit     = s.limit_pressed ? 1 : 0;
    st.fault     = s.fault_bits;
    st.driver    = s.driver_requested_on ? 1 : 0;
    annaLinkSendStatus(st);
  }
}

static void stepperTask(void*) {
  for (;;) {
    bool busy = annaDriverServiceStepper();
    if (!busy) {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

static void sensorTask(void*) {
  // Encoder sampling is only used for UI/visualisation; keep it low priority and low bandwidth.
  const TickType_t period = pdMS_TO_TICKS(20);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, period);
    if (!g_encoder_enabled) {
      vTaskDelete(nullptr);
    }
    if (!g_as_ok) {
      axisStateSetEncoderOk(false);
      vTaskDelete(nullptr);
    }
    uint16_t raw = g_as5600.readAngle();
    int err = g_as5600.lastError();
    if (err != AS5600_OK) {
      if (g_as_error_streak < UINT16_MAX) ++g_as_error_streak;
      if (g_as_error_streak >= 25) {
        (void)recoverAs5600(millis());
      }
      continue;
    }
    g_as_error_streak = 0;
    processAs5600Raw(raw);
  }
}

static void tempTask(void*) {
  static constexpr uint32_t kTempSampleMs = 2000;
  static constexpr uint32_t kConvMs = 750;

  const TickType_t period = pdMS_TO_TICKS(kTempSampleMs);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);
    if (!g_temp_ok) {
      axisStateSetTemperature(NAN, false);
      continue;
    }
    g_ds18b20.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(kConvMs));
    float reading = g_ds18b20.getTempCByIndex(0);
    bool ok = (reading > -55.0f && reading < 125.0f);
    axisStateSetTemperature(reading, ok);
  }
}

static void tempDetectTask(void*) {
  static constexpr uint32_t kTempRetryDelayMs = 10000;
  static constexpr uint8_t kTempRetryCount = 3;

  for (uint8_t attempt = 0; attempt < kTempRetryCount; ++attempt) {
    vTaskDelay(pdMS_TO_TICKS(kTempRetryDelayMs));
    if (g_temp_ok) break;
    g_ds18b20.begin();
    g_ds18b20.setWaitForConversion(false);
    g_temp_ok = (g_ds18b20.getDeviceCount() > 0);
    if (g_temp_ok) {
      axisStateSetTemperature(NAN, false);
      if (!g_h_tmp) {
        xTaskCreatePinnedToCore(tempTask, "tmp0p5", 3072, nullptr, 1, &g_h_tmp, kCoreHeavy);
      }
      break;
    }
  }
  g_h_tmp_retry = nullptr;
  vTaskDelete(nullptr);
}
} // namespace

void annaTasksInitPeripherals() {
  // DS18B20
  g_ds18b20.begin();
  g_ds18b20.setWaitForConversion(false);
  g_temp_ok = (g_ds18b20.getDeviceCount() > 0);
  axisStateSetTemperature(NAN, false);

  // Limit switch
  g_limit_btn.attach(PIN_LIMIT, INPUT_PULLUP);
  g_limit_btn.interval(3);
  axisStateSetLimitPressed(false);

  // Local homing button (active-low, pulled up).
  g_home_btn.attach(PIN_HOMING_BTN, INPUT_PULLUP);
  g_home_btn.interval(kHomeDebounceMs);
}

void annaTasksStart() {
  xTaskCreatePinnedToCore(stepperTask, "stp", 4096, nullptr, 4, &g_h_stp, kCoreStepper);

  xTaskCreatePinnedToCore(controlTask, "ctl1k", 4096, nullptr, 3, &g_h_ctl, kCoreHeavy);
  xTaskCreatePinnedToCore(linkTask,    "lnk1k", 4096, nullptr, 2, &g_h_link, kCoreHeavy);
  xTaskCreatePinnedToCore(annaUiTask,  "ui20h", 4096, nullptr, 1, &g_h_ui, kCoreHeavy);
  if (g_temp_ok) {
    xTaskCreatePinnedToCore(tempTask,    "tmp0p5", 3072, nullptr, 1, &g_h_tmp, kCoreHeavy);
  } else if (!g_h_tmp_retry) {
    xTaskCreatePinnedToCore(tempDetectTask, "tmpdet", 3072, nullptr, 1, &g_h_tmp_retry, kCoreHeavy);
  }
}

void annaTasksConfigureAxisR() {
  if (g_axis_is_r) return;
  g_axis_is_r = true;
  g_encoder_enabled = false;
  g_as_ok = false;
  g_encoder_configured = false;
  if (g_h_sns) {
    vTaskDelete(g_h_sns);
    g_h_sns = nullptr;
  }
  g_wire_encoder.end();
  axisStateSetEncoderOk(false);
  axisStateSetKinematics(0.0f, 0.0f);
}

void annaTasksConfigureAxisNonR() {
  if (g_axis_is_r || g_encoder_configured) return;
  g_encoder_configured = true;
  g_encoder_enabled = true;

  g_wire_encoder.begin(I2C_ENCODER_SDA_PIN, I2C_ENCODER_SCL_PIN, I2C_ENCODER_CLOCK_HZ);
  g_wire_encoder.setClock(I2C_ENCODER_CLOCK_HZ);
  g_wire_encoder.setTimeOut(50);

  g_as_ok = g_as5600.begin();
  axisStateSetEncoderOk(g_as_ok);
  if (g_as_ok) {
    configureAs5600ForBandwidth();
  } else {
    g_encoder_enabled = false;
  }

  pinMode(PIN_AS5600_DIR, OUTPUT);
  digitalWrite(PIN_AS5600_DIR, LOW);

  // Seed unwrap pipeline.
  if (g_as_ok) {
    g_last_raw = g_as5600.readAngle();
  } else {
    g_last_raw = 0;
  }
  g_turns = 0;
  g_last_pos_deg = g_last_raw * (360.0f / 4096.0f);
  g_last_vel_us = micros();
  axisStateSetKinematics(g_last_pos_deg, 0.0f);

  if (g_as_ok && !g_h_sns) {
    xTaskCreatePinnedToCore(sensorTask,  "sns1k", 3072, nullptr, 2, &g_h_sns, kCoreHeavy);
  }
}
