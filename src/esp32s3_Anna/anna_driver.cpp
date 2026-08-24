#include "anna_driver.h"

#include <AccelStepper.h>
#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "anna_link.h"
#include "anna_pins.h"
#include "anna_state.h"

namespace {
HardwareSerial& kTmcUart = Serial2;

static constexpr const char* kPrefsNamespace     = "axis_cfg";
static constexpr const char* kPrefsStandstillKey = "standstill";

static constexpr uint16_t kMicrostepsDefault = 32;

static constexpr long kTmcBaud = 57600;
static constexpr TMC2209::SerialAddress kTmcSerialAddress = TMC2209::SERIAL_ADDRESS_0;

// Tuning: prioritize cool/quiet operation.
static constexpr uint8_t kRunCurrentDefaultPct = 30;
static constexpr uint8_t kHoldCurrentMaxPct    = 10;
static constexpr uint8_t kHoldDelayPct         = 0;  // drop to hold quickly after standstill
static constexpr uint8_t kPowerDownDelay       = 2;  // minimum recommended for StealthChop auto-tuning

	// AccelStepper drives the STEP/DIR pins directly. Speed units are "microsteps/s" (one STEP pulse per microstep),
	// so host velocity commands (full-steps/s) must be scaled by the active microstep setting.
	static constexpr uint16_t kAccelStepperMinPulseWidthUs = 2;
	static constexpr float    kAccelStepperMaxSps = 1000.0f;  // requested max speed (full steps/s)
	static constexpr float    kAccelStepperMaxUstepsPerSecHard = 50000.0f; // hard cap for pulse generation safety

	TMC2209 g_driver;
	QueueHandle_t g_req_q = nullptr;

	uint16_t g_microsteps = kMicrostepsDefault;         // last-known/applied
	uint16_t g_desired_microsteps = kMicrostepsDefault; // desired/guarded
	uint8_t  g_standstill_mode = (uint8_t)TMC2209::NORMAL;
	uint8_t  g_standstill_mode_applied = (uint8_t)TMC2209::NORMAL;
	uint8_t  g_run_current_pct = kRunCurrentDefaultPct;
	bool     g_direction_inverted = false;

enum class VelocityKind : uint8_t { Vactual, StepsPerSecond, MoveSteps };
VelocityKind g_vel_kind = VelocityKind::StepsPerSecond;
float   g_vel_sps = 0.0f;
int32_t g_vel_vactual = 0;

	AccelStepper* g_stepper = nullptr;
	portMUX_TYPE g_stepper_mux = portMUX_INITIALIZER_UNLOCKED;
	float g_stepper_target_usteps_per_s = 0.0f;
	bool  g_stepper_active = false; // only true when using the AccelStepper backend and driver is enabled
	enum class StepperMode : uint8_t { Speed, Move };
	StepperMode g_stepper_mode = StepperMode::Speed;
	bool  g_stepper_move_active = false;

	// Relative position ("RP") tracking.
	// AccelStepper position units are STEP pulses (microsteps). We accumulate in full steps so the result is
	// meaningful to the host console and stable even if microstepping is changed at runtime.
	double  g_pos_full_steps = 0.0;      // open-loop full-step position since boot
	double  g_rp_zero_full_steps = 0.0;  // zero reference captured at boot / limit press
	int32_t g_last_pos_usteps = 0;        // last observed AccelStepper position (microsteps)
	bool    g_have_pos_usteps = false;
	int32_t g_rp_steps_cached = 0;        // rounded RP (full steps) for fast reads from other tasks

		uint32_t g_move_seq = 0;
		int32_t  g_move_steps = 0;       // full steps (not microsteps)
		float    g_move_vel_sps = 0.0f;  // full-steps/s (magnitude)
		float    g_move_accel_sps2 = 0.0f; // full-steps/s^2 (magnitude)
		bool     g_move_abort = false;
		int8_t   g_move_dir = 0; // cached sign of last requested move (+1 / -1 / 0)
		uint32_t g_move_done_seq = 0; // last move_seq that completed (latched by stepper task)

	portMUX_TYPE g_driver_mux = portMUX_INITIALIZER_UNLOCKED;

	// NOTE: Holding torque is required at rest, so we do not auto-freewheel on idle.
	// (Set to non-zero only if you explicitly want coils to drop after an idle timeout.)
	static constexpr uint32_t kIdleFreewheelAfterMs = 0;

	static uint8_t sanitizeStandstillMode(uint8_t raw) {
	  switch (raw) {
	    case (uint8_t)TMC2209::NORMAL:
    case (uint8_t)TMC2209::FREEWHEELING:
    case (uint8_t)TMC2209::BRAKING:
    case (uint8_t)TMC2209::STRONG_BRAKING:
      return raw;
    default:
      return (uint8_t)TMC2209::NORMAL;
	  }
	}

	static void applyStandstillModeHardwareOnly(uint8_t mode) {
	  uint8_t sanitized = sanitizeStandstillMode(mode);
	  portENTER_CRITICAL(&g_driver_mux);
	  g_standstill_mode_applied = sanitized;
	  portEXIT_CRITICAL(&g_driver_mux);
	  g_driver.setStandstillMode(static_cast<TMC2209::StandstillMode>(sanitized));
	}

	static void applyStandstillMode(uint8_t mode) {
	  uint8_t sanitized = sanitizeStandstillMode(mode);
	  portENTER_CRITICAL(&g_driver_mux);
	  g_standstill_mode = sanitized;
	  portEXIT_CRITICAL(&g_driver_mux);
	  applyStandstillModeHardwareOnly(sanitized);
	}

	static void applyDirectionInverted() {
	  if (g_stepper) {
	    g_stepper->setPinsInverted(g_direction_inverted, false, false);
	  }
	}

static void loadStandstillMode() {
  Preferences prefs;
  uint8_t stored = (uint8_t)TMC2209::NORMAL;
  if (prefs.begin(kPrefsNamespace, true)) {
    stored = prefs.getUChar(kPrefsStandstillKey, stored);
    prefs.end();
  }
  uint8_t sanitized = sanitizeStandstillMode(stored);
  applyStandstillMode(sanitized);
  if (sanitized != stored) {
    annaDriverPersistStandstillMode(sanitized);
  }
}

static inline void motorEnable(bool en) {
  static bool last = false;
  if (en == last) return;
  last = en;
  if (en) {
    g_driver.enable();
  } else {
    g_driver.disable();
    digitalWrite(PIN_TMC_STEP, LOW);
  }
}

			static inline float clampAbs(float v, float limit) {
			  if (!isfinite(v) || limit <= 0.0f) return v;
			  if (v > limit) return limit;
			  if (v < -limit) return -limit;
		  return v;
	}

	static inline uint16_t readMicrostepsOrDefault() {
	  portENTER_CRITICAL(&g_driver_mux);
	  uint16_t ms = g_microsteps;
	  portEXIT_CRITICAL(&g_driver_mux);
	  if (ms == 0) ms = kMicrostepsDefault;
	  return ms;
	}

	static inline void setStepperTarget(float usteps_per_s, bool active, StepperMode mode) {
	  portENTER_CRITICAL(&g_stepper_mux);
	  g_stepper_target_usteps_per_s = usteps_per_s;
	  g_stepper_active = active;
	  g_stepper_mode = mode;
	  portEXIT_CRITICAL(&g_stepper_mux);
	}

static void sendLine(const char* fmt, ...) {
  char line[128];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  if (n <= 0) return;
  annaLinkSendEvent(reinterpret_cast<const uint8_t*>(line), (uint16_t)min(n, (int)sizeof(line)));
}

static inline uint8_t computeHoldCurrentPct(uint8_t run_pct) {
  return (uint8_t)min((int)run_pct, (int)kHoldCurrentMaxPct);
}

		static void applyTmcCoolConfig() {
		  const uint8_t run_pct = (uint8_t)constrain((int)g_run_current_pct, 0, 100);
		  const uint8_t hold_pct = computeHoldCurrentPct(run_pct);

	  // Keep current low and predictable.
	  g_driver.setAllCurrentValues(run_pct, hold_pct, kHoldDelayPct);
	  g_driver.setPowerDownDelay(kPowerDownDelay);

	  // Current control mode (autoscale) keeps actual coil current closer to target (less heat drift).
	  g_driver.enableAutomaticCurrentScaling();
	  g_driver.enableAutomaticGradientAdaptation();

	  // Quiet mode. We leave thresholds at library defaults; mode selection is via GCONF.
	  g_driver.enableStealthChop();

	  // Keep torque/current consistent. (CoolStep can introduce audible "roughness" without StallGuard tuning.)
	  g_driver.disableCoolStep();

		  // Re-assert microstep settings (some fault/reset cases can revert registers).
		  g_driver.setMicrostepsPerStep(g_desired_microsteps);
		  const uint16_t applied_microsteps = g_driver.getMicrostepsPerStep();
		  portENTER_CRITICAL(&g_driver_mux);
	  g_microsteps = applied_microsteps;
	  portEXIT_CRITICAL(&g_driver_mux);

	  // Re-assert standstill behavior (loaded from NVS and/or host command).
	  uint8_t desired_mode = (uint8_t)TMC2209::NORMAL;
	  portENTER_CRITICAL(&g_driver_mux);
	  desired_mode = g_standstill_mode;
	  portEXIT_CRITICAL(&g_driver_mux);
	  applyStandstillModeHardwareOnly(desired_mode);
	}


	static void handleRequest(const DriverRequest& req) {
	  switch (req.type) {
	    case DriverRequestType::SetRunCurrentPct:
	      g_run_current_pct = (uint8_t)constrain((int)req.i32, 0, 100);
	      applyTmcCoolConfig();
	      break;
    case DriverRequestType::SetMicrosteps: {
      uint16_t requested = req.u16;
      if (requested == 0) requested = kMicrostepsDefault;
      g_driver.setMicrostepsPerStep(requested);
      uint16_t applied = g_driver.getMicrostepsPerStep();
      g_desired_microsteps = applied;
      portENTER_CRITICAL(&g_driver_mux);
      g_microsteps = applied;
      portEXIT_CRITICAL(&g_driver_mux);
      if (applied != requested) {
        sendLine("tmc.microsteps applied: req=%u applied=%u", (unsigned)requested, (unsigned)applied);
      }
      break;
    }
	    case DriverRequestType::SetStandstillMode:
	      applyStandstillMode(req.u8);
	      break;
	    case DriverRequestType::SetVelocitySps:
	      g_vel_kind = VelocityKind::StepsPerSecond;
	      g_vel_sps = req.f32;
	      break;
	    case DriverRequestType::SetVactual:
	      // Deprecated: keep STEP/DIR as the only motion backend.
	      // Interpret VACTUAL as 0 when received to avoid accidentally switching to UART-driven motion.
	      g_vel_kind = VelocityKind::StepsPerSecond;
	      g_vel_sps = 0.0f;
	      g_vel_vactual = req.i32;
	      break;
	    case DriverRequestType::MoveSteps: {
	      const int32_t steps = req.i32;
	      if (steps == 0) break;

      const float vel_sps = fabsf(req.f32);
      float accel_sps2 = fabsf(req.f32b);
      if (!isfinite(vel_sps) || vel_sps <= 0.0f) {
        sendLine("move.reject: velocity invalid");
        break;
      }
      if (!isfinite(accel_sps2) || accel_sps2 <= 0.0f) {
        // Default accel: quick ramp (~0.25s to reach cruise) without relying on host motion caps.
        accel_sps2 = max(10.0f, vel_sps * 4.0f);
      }

      g_vel_kind = VelocityKind::MoveSteps;
      g_vel_sps = 0.0f;
      g_vel_vactual = 0;

      g_move_dir = (steps > 0) ? +1 : -1;
	      portENTER_CRITICAL(&g_stepper_mux);
	      g_move_steps = steps;
	      g_move_vel_sps = vel_sps;
	      g_move_accel_sps2 = accel_sps2;
	      ++g_move_seq;
	      g_move_done_seq = 0;
	      g_move_abort = false;
	      portEXIT_CRITICAL(&g_stepper_mux);

	      sendLine("-> move start steps=%ld vel=%.1f accel=%.1f",
	               (long)steps,
	               vel_sps,
	               accel_sps2);
	      break;
	    }
    case DriverRequestType::DumpSettings: {
      sendLine("*************************");
      sendLine("getSettings()");
      TMC2209::Settings settings = g_driver.getSettings();
      sendLine("settings.is_communicating = %u", (unsigned)settings.is_communicating);
      sendLine("settings.is_setup = %u", (unsigned)settings.is_setup);
      sendLine("settings.software_enabled = %u", (unsigned)settings.software_enabled);
      sendLine("settings.microsteps_per_step = %u", (unsigned)settings.microsteps_per_step);
      sendLine("settings.inverse_motor_direction_enabled = %u", (unsigned)settings.inverse_motor_direction_enabled);
      sendLine("settings.stealth_chop_enabled = %u", (unsigned)settings.stealth_chop_enabled);
      switch (settings.standstill_mode) {
        case TMC2209::NORMAL:         sendLine("settings.standstill_mode = normal"); break;
        case TMC2209::FREEWHEELING:   sendLine("settings.standstill_mode = freewheeling"); break;
        case TMC2209::STRONG_BRAKING: sendLine("settings.standstill_mode = strong_braking"); break;
        case TMC2209::BRAKING:        sendLine("settings.standstill_mode = braking"); break;
        default:                      sendLine("settings.standstill_mode = (unknown)"); break;
      }
      sendLine("settings.irun_percent = %u", (unsigned)settings.irun_percent);
      sendLine("settings.irun_register_value = %u", (unsigned)settings.irun_register_value);
      sendLine("settings.ihold_percent = %u", (unsigned)settings.ihold_percent);
      sendLine("settings.ihold_register_value = %u", (unsigned)settings.ihold_register_value);
      sendLine("settings.iholddelay_percent = %u", (unsigned)settings.iholddelay_percent);
      sendLine("settings.iholddelay_register_value = %u", (unsigned)settings.iholddelay_register_value);
      sendLine("settings.automatic_current_scaling_enabled = %u", (unsigned)settings.automatic_current_scaling_enabled);
      sendLine("settings.automatic_gradient_adaptation_enabled = %u", (unsigned)settings.automatic_gradient_adaptation_enabled);
      sendLine("settings.pwm_offset = %u", (unsigned)settings.pwm_offset);
      sendLine("settings.pwm_gradient = %u", (unsigned)settings.pwm_gradient);
      sendLine("settings.cool_step_enabled = %u", (unsigned)settings.cool_step_enabled);
      sendLine("settings.analog_current_scaling_enabled = %u", (unsigned)settings.analog_current_scaling_enabled);
      sendLine("settings.internal_sense_resistors_enabled = %u", (unsigned)settings.internal_sense_resistors_enabled);
      sendLine("*************************");
      break;
    }
    case DriverRequestType::DumpStatus: {
      sendLine("*************************");
      sendLine("hardwareDisabled()");
      const bool hardware_disabled = g_driver.hardwareDisabled();
      sendLine("hardware_disabled = %u", (unsigned)hardware_disabled);
      sendLine("*************************");
      sendLine("*************************");
      sendLine("getStatus()");
      TMC2209::Status status = g_driver.getStatus();
      sendLine("status.over_temperature_warning = %u", (unsigned)status.over_temperature_warning);
      sendLine("status.over_temperature_shutdown = %u", (unsigned)status.over_temperature_shutdown);
      sendLine("status.short_to_ground_a = %u", (unsigned)status.short_to_ground_a);
      sendLine("status.short_to_ground_b = %u", (unsigned)status.short_to_ground_b);
      sendLine("status.low_side_short_a = %u", (unsigned)status.low_side_short_a);
      sendLine("status.low_side_short_b = %u", (unsigned)status.low_side_short_b);
      sendLine("status.open_load_a = %u", (unsigned)status.open_load_a);
      sendLine("status.open_load_b = %u", (unsigned)status.open_load_b);
      sendLine("status.over_temperature_120c = %u", (unsigned)status.over_temperature_120c);
      sendLine("status.over_temperature_143c = %u", (unsigned)status.over_temperature_143c);
      sendLine("status.over_temperature_150c = %u", (unsigned)status.over_temperature_150c);
      sendLine("status.over_temperature_157c = %u", (unsigned)status.over_temperature_157c);
      sendLine("status.current_scaling = %u", (unsigned)status.current_scaling);
      sendLine("status.stealth_chop_mode = %u", (unsigned)status.stealth_chop_mode);
      sendLine("status.standstill = %u", (unsigned)status.standstill);
      sendLine("*************************");
      break;
    }
    default:
      break;
  }
}
} // namespace

	void annaDriverInit() {
  pinMode(PIN_TMC_EN, OUTPUT);
  // Disable driver until UART + current limits are configured.
  digitalWrite(PIN_TMC_EN, HIGH); // active-low enable
  pinMode(PIN_TMC_MS1, OUTPUT);
  pinMode(PIN_TMC_MS2, OUTPUT);
  pinMode(PIN_TMC_STEP, OUTPUT);
  pinMode(PIN_TMC_DIR, OUTPUT);
  pinMode(PIN_TMC_INDEX, INPUT);
  pinMode(PIN_TMC_DIAG, INPUT_PULLUP); // active-low

  static constexpr int MCU_TMC_RX_PIN = PIN_TMC_TX; // MCU RX <- PDN/UART
  static constexpr int MCU_TMC_TX_PIN = PIN_TMC_RX; // MCU TX -> PDN/UART (via 1k)

  g_driver.setup(kTmcUart, kTmcBaud, kTmcSerialAddress, MCU_TMC_RX_PIN, MCU_TMC_TX_PIN);
  g_driver.setReplyDelay(15);
  g_driver.setHardwareEnablePin(PIN_TMC_EN);

	  static AccelStepper stepper(AccelStepper::DRIVER, PIN_TMC_STEP, PIN_TMC_DIR);
	  g_stepper = &stepper;
	  stepper.setMinPulseWidth(kAccelStepperMinPulseWidthUs);
	  stepper.setMaxSpeed(kAccelStepperMaxSps * (float)kMicrostepsDefault);
	  stepper.setSpeed(0.0f);
	  applyDirectionInverted();

  g_desired_microsteps = kMicrostepsDefault;
  g_run_current_pct = kRunCurrentDefaultPct;
  applyTmcCoolConfig();
  g_microsteps = g_driver.getMicrostepsPerStep();

  loadStandstillMode();

  auto tmc_settings = g_driver.getSettings();
  Serial.printf("TMC2209: ver=0x%02X comm=%d setup=%d addr=%d baud=%ld\n",
                g_driver.getVersion(),
                (int)tmc_settings.is_communicating,
                (int)tmc_settings.is_setup,
                (int)kTmcSerialAddress,
                kTmcBaud);
  if (!tmc_settings.is_communicating) {
    Serial.println("TMC2209: not communicating, rebooting ESP32-S3");
    delay(50);
    ESP.restart();
  }

  g_driver.moveAtVelocity(0);
  motorEnable(true);
  setStepperTarget(0.0f, false, StepperMode::Speed);

  if (!g_req_q) g_req_q = xQueueCreate(16, sizeof(DriverRequest));
}

bool annaDriverOk() {
  auto tmc_settings = g_driver.getSettings();
  return tmc_settings.is_communicating;
}

bool annaDriverEnqueue(const DriverRequest& req) {
  if (!g_req_q) return false;
  return xQueueSend(g_req_q, &req, 0) == pdTRUE;
}

void annaDriverSetDirectionInverted(bool inverted) {
  g_direction_inverted = inverted;
  applyDirectionInverted();
}

			void annaDriverTick(uint32_t now_ms, bool link_ok, bool enable_requested, bool limit_pressed) {
		  // Drain queued requests (non-blocking).
		  if (g_req_q) {
		    DriverRequest req;
		    while (xQueueReceive(g_req_q, &req, 0) == pdTRUE) {
		      handleRequest(req);
	    }
	  }

	  // When the limit switch is hit (or held at boot), reset RP to 0.
	  static bool last_limit_pressed = false;
	  if (limit_pressed && !last_limit_pressed) {
	    portENTER_CRITICAL(&g_stepper_mux);
	    g_rp_zero_full_steps = g_pos_full_steps;
	    g_rp_steps_cached = 0;
	    portEXIT_CRITICAL(&g_stepper_mux);
	  }
	  last_limit_pressed = limit_pressed;

		  // Link safety: force stop on disconnect.
		  if (!link_ok) {
		    g_vel_kind = VelocityKind::StepsPerSecond;
	    g_vel_sps = 0.0f;
		    g_vel_vactual = 0;
	    g_move_dir = 0;
	    portENTER_CRITICAL(&g_stepper_mux);
	    g_move_abort = true;
	    portEXIT_CRITICAL(&g_stepper_mux);
	  }

		  float desired_sps = (g_vel_kind == VelocityKind::StepsPerSecond) ? g_vel_sps : 0.0f;

  // Limit safety: when pressed, block motion further into the limit switch (positive steps/velocity).
  if (limit_pressed) {
    if (g_vel_kind == VelocityKind::StepsPerSecond) {
      if (desired_sps > 0.0f) desired_sps = 0.0f;
	    } else if (g_vel_kind == VelocityKind::Vactual) {
	      // Deprecated backend (keep STEP/DIR only). Force stop by clearing the cached vactual.
	      g_vel_vactual = 0;
	    } else if (g_vel_kind == VelocityKind::MoveSteps) {
	      // For open-loop step moves, treat "positive steps" as the forbidden direction when the limit is pressed,
	      // matching the existing velocity-mode safety rule above.
	      if (g_move_dir > 0) {
	        g_move_dir = 0;
        portENTER_CRITICAL(&g_stepper_mux);
        g_move_abort = true;
        portEXIT_CRITICAL(&g_stepper_mux);
      }
    }
  }

		  const bool enable_eff = enable_requested && link_ok;
		  motorEnable(enable_eff);

	  if (kIdleFreewheelAfterMs != 0) {
	    // Optional idle heat reduction (disabled by default). If enabled, this drops coils after a true idle
	    // timeout, which removes holding torque. Leave at 0 if holding torque is required.
	    static uint32_t last_motion_ms = 0;
	    if (last_motion_ms == 0) last_motion_ms = now_ms;

	    bool move_active = false;
	    if (g_vel_kind == VelocityKind::MoveSteps) {
	      portENTER_CRITICAL(&g_stepper_mux);
	      move_active = g_stepper_move_active;
	      portEXIT_CRITICAL(&g_stepper_mux);
	    }
	    bool motion_commanded = false;
		    if (g_vel_kind == VelocityKind::StepsPerSecond) {
		      motion_commanded = (desired_sps != 0.0f);
		    } else if (g_vel_kind == VelocityKind::Vactual) {
		      motion_commanded = false;
		    } else {
		      motion_commanded = move_active;
		    }

	    if (!enable_eff || motion_commanded) {
	      last_motion_ms = now_ms;
	    }

	    uint8_t desired_mode = (uint8_t)TMC2209::NORMAL;
	    uint8_t applied_mode = (uint8_t)TMC2209::NORMAL;
	    portENTER_CRITICAL(&g_driver_mux);
	    desired_mode = g_standstill_mode;
	    applied_mode = g_standstill_mode_applied;
	    portEXIT_CRITICAL(&g_driver_mux);

	    const bool should_freewheel =
	      enable_eff &&
	      !motion_commanded &&
	      ((uint32_t)(now_ms - last_motion_ms) >= kIdleFreewheelAfterMs) &&
	      (desired_mode == (uint8_t)TMC2209::NORMAL);

	    const uint8_t target_mode =
	      should_freewheel ? (uint8_t)TMC2209::FREEWHEELING : desired_mode;
	    if (applied_mode != target_mode) {
	      applyStandstillModeHardwareOnly(target_mode);
	    }
	  }

	  if (!enable_eff && g_vel_kind == VelocityKind::MoveSteps) {
	    // Don't allow a queued move to resume automatically after re-enable.
	    g_move_dir = 0;
	    portENTER_CRITICAL(&g_stepper_mux);
    g_move_abort = true;
    portEXIT_CRITICAL(&g_stepper_mux);
  }

			  if (g_vel_kind == VelocityKind::StepsPerSecond) {
			    uint16_t microsteps = 0;
			    portENTER_CRITICAL(&g_driver_mux);
			    microsteps = g_microsteps;
		    portEXIT_CRITICAL(&g_driver_mux);
	    if (microsteps == 0) microsteps = kMicrostepsDefault;
	    float usteps_per_s = enable_eff ? (desired_sps * (float)microsteps) : 0.0f;
	    if (!isfinite(usteps_per_s)) usteps_per_s = 0.0f;
	    const float usteps_limit = min(kAccelStepperMaxUstepsPerSecHard, kAccelStepperMaxSps * (float)microsteps);
	    usteps_per_s = clampAbs(usteps_per_s, usteps_limit);
		    setStepperTarget(usteps_per_s, enable_eff, StepperMode::Speed);
		  } else if (g_vel_kind == VelocityKind::MoveSteps) {
		    // AccelStepper position moves are executed in annaDriverServiceStepper().
		    setStepperTarget(0.0f, enable_eff, StepperMode::Move);
			  } else {
			    // Unknown/legacy backend: stop.
			    setStepperTarget(0.0f, false, StepperMode::Speed);
			  }

		  // Emit any "move done" notifications from the 1 kHz control task (never from the stepper task).
		  uint32_t done_seq = 0;
		  portENTER_CRITICAL(&g_stepper_mux);
		  done_seq = g_move_done_seq;
		  g_move_done_seq = 0;
		  portEXIT_CRITICAL(&g_stepper_mux);
		  if (done_seq != 0) {
		    sendLine("-> move done");
		  }

		}

	bool annaDriverServiceStepper() {
	  if (!g_stepper) return false;

	  float target_usteps_per_s = 0.0f;
	  bool active = false;
	  StepperMode mode = StepperMode::Speed;

  uint32_t move_seq = 0;
  int32_t move_steps = 0;
  float move_vel_sps = 0.0f;
  float move_accel_sps2 = 0.0f;
  bool abort_move = false;

  portENTER_CRITICAL(&g_stepper_mux);
  target_usteps_per_s = g_stepper_target_usteps_per_s;
  active = g_stepper_active;
  mode = g_stepper_mode;
  move_seq = g_move_seq;
  move_steps = g_move_steps;
  move_vel_sps = g_move_vel_sps;
  move_accel_sps2 = g_move_accel_sps2;
	  abort_move = g_move_abort;
	  g_move_abort = false; // consume abort latch
	  portEXIT_CRITICAL(&g_stepper_mux);

	  bool busy = false;

		  static uint32_t last_move_seq_applied = 0;
		  static uint32_t last_move_seq_reported_done = 0;

		  if (mode == StepperMode::Speed) {
	    portENTER_CRITICAL(&g_stepper_mux);
	    g_stepper_move_active = false;
	    portEXIT_CRITICAL(&g_stepper_mux);

		    float target = target_usteps_per_s;
		    if (!active) target = 0.0f;
		    if (!isfinite(target)) target = 0.0f;

		    busy = active && (target != 0.0f);

	    static float last_set = NAN;
		    if (!isfinite(last_set) || target != last_set) {
		      g_stepper->setSpeed(target);
		      last_set = target;
	    }
	    (void)g_stepper->runSpeed();
	  } else {
	    // Move mode (AccelStepper position control).
	    if (abort_move) {
	      // Immediate stop: zero distance-to-go without generating any further steps.
	      g_stepper->moveTo(g_stepper->currentPosition());
	      g_stepper->setSpeed(0.0f);
	    }

	    if (move_seq != 0 && move_seq != last_move_seq_applied) {
	      last_move_seq_applied = move_seq;
	      last_move_seq_reported_done = 0;

	      uint16_t microsteps = readMicrostepsOrDefault();

	      int64_t rel_usteps64 = (int64_t)move_steps * (int64_t)microsteps;
	      if (rel_usteps64 > (int64_t)INT32_MAX) rel_usteps64 = (int64_t)INT32_MAX;
	      if (rel_usteps64 < (int64_t)INT32_MIN) rel_usteps64 = (int64_t)INT32_MIN;
	      int32_t rel_usteps = (int32_t)rel_usteps64;

	      float max_usteps_per_s = fabsf(move_vel_sps) * (float)microsteps;
	      float accel_usteps_per_s2 = fabsf(move_accel_sps2) * (float)microsteps;
	      if (!isfinite(max_usteps_per_s) || max_usteps_per_s <= 0.0f) max_usteps_per_s = 1.0f;
	      if (!isfinite(accel_usteps_per_s2) || accel_usteps_per_s2 <= 0.0f) accel_usteps_per_s2 = 1.0f;
	      const float usteps_limit = min(kAccelStepperMaxUstepsPerSecHard, kAccelStepperMaxSps * (float)microsteps);
	      if (max_usteps_per_s > usteps_limit) max_usteps_per_s = usteps_limit;

	      g_stepper->setMaxSpeed(max_usteps_per_s);
	      g_stepper->setAcceleration(accel_usteps_per_s2);
	      g_stepper->setSpeed(0.0f);
	      g_stepper->move((long)rel_usteps);

		    }

		    if (active) {
		      (void)g_stepper->run();
		    }

		    const bool move_active = g_stepper->distanceToGo() != 0;
		    busy = active && move_active;
		    portENTER_CRITICAL(&g_stepper_mux);
		    g_stepper_move_active = move_active;
		    portEXIT_CRITICAL(&g_stepper_mux);

		    if (move_seq != 0 &&
		        last_move_seq_reported_done != move_seq &&
		        g_stepper->distanceToGo() == 0) {
		      last_move_seq_reported_done = move_seq;
		      portENTER_CRITICAL(&g_stepper_mux);
		      g_move_done_seq = move_seq;
		      portEXIT_CRITICAL(&g_stepper_mux);
		    }
		  }

	  // Publish RP based on actual generated STEP pulses.
	  const int32_t pos_usteps = (int32_t)g_stepper->currentPosition();
	  const uint16_t microsteps = readMicrostepsOrDefault();

	  portENTER_CRITICAL(&g_stepper_mux);
	  if (!g_have_pos_usteps) {
	    g_have_pos_usteps = true;
	    g_last_pos_usteps = pos_usteps;
	  } else {
	    const int32_t delta_usteps = pos_usteps - g_last_pos_usteps;
	    g_last_pos_usteps = pos_usteps;
	    if (delta_usteps != 0 && microsteps != 0) {
	      g_pos_full_steps += (double)delta_usteps / (double)microsteps;
	    }
	  }

	  double rp_steps = g_pos_full_steps - g_rp_zero_full_steps;
	  if (rp_steps > (double)INT32_MAX) rp_steps = (double)INT32_MAX;
	  if (rp_steps < (double)INT32_MIN) rp_steps = (double)INT32_MIN;
	  g_rp_steps_cached = (int32_t)lround(rp_steps);
		  portEXIT_CRITICAL(&g_stepper_mux);

		  return busy;
		}

uint16_t annaDriverMicrosteps() {
  portENTER_CRITICAL(&g_driver_mux);
  uint16_t ms = g_microsteps;
  portEXIT_CRITICAL(&g_driver_mux);
  return ms;
}

uint8_t annaDriverStandstillMode() {
  portENTER_CRITICAL(&g_driver_mux);
  uint8_t mode = g_standstill_mode;
  portEXIT_CRITICAL(&g_driver_mux);
  return mode;
}

	int32_t annaDriverRelativePositionSteps() {
	  portENTER_CRITICAL(&g_stepper_mux);
	  int32_t v = g_rp_steps_cached;
	  portEXIT_CRITICAL(&g_stepper_mux);
	  return v;
	}

	void annaDriverPersistStandstillMode(uint8_t mode) {
	  Preferences prefs;
	  if (prefs.begin(kPrefsNamespace, false)) {
	    prefs.putUChar(kPrefsStandstillKey, sanitizeStandstillMode(mode));
    prefs.end();
  }
}
