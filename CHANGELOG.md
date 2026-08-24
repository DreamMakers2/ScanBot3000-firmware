# Changelog

## 2026-01-27
- `lib/proto`: added `MeasureRange` (60) and `SetPixels` (61) command IDs, the 8-LED `LedFrame` payload, and `sendCommandPayload` for variable-length command payloads.
- `teensy41_Beata`: CLI `led` now sends packed LED frames with optional fade (`T=`) and brightness (`B=`), and a new `measure <axis> <seconds>` command triggers VL6180X sampling on axis R.
- `esp32s3_Anna`: axis identity now configures encoder vs. range mode after `setname`; axis R disables the AS5600 I2C bus and repurposes GPIO12/13 for VL6180X SHUT plus an activity NeoPixel.
- `esp32s3_Anna`: `measure` sampling (10 Hz) powers the VL6180X on demand, updates the OLED with large range numerals, drives a rainbow activity LED, emits `range_mm:`/`range.err=` event lines, and mirrors valid ranges into `dist_mm` telemetry.
- `esp32s3_Anna`: LED UI now accepts `LedFrame` overrides with per-LED masks, optional brightness updates, and optional fades; stop alerts display "Limit Triggered" with a red sweep; the temperature task now starts only when a DS18B20 probe is detected.
- `esp32s3_Anna`: OLED AP is hidden when the AS5600 is missing, the DS18B20 probe is rechecked after boot (10-second delay, up to 3 retries), LED0 link status resumes 5 seconds after the last LED command, and the measure-mode rainbow animation runs 2x faster.
- `platformio.ini`: added the local `Adafruit_VL6180X` dependency for axis R range sensing.

## 2025-12-19
- Removed the `esp32s3_DriverTest` PlatformIO environment (and its sources); the project now only builds `teensy41_Beata` and `esp32s3_Anna`.
- `esp32s3_Anna`: TMC2209 UART defaults updated (baud `57600`, standstill fallback/default `normal`), and the device now reboots on boot if the driver is not communicating.
- `esp32s3_Anna`: removed background TMC2209 status polling; `tmcsettings`/`tmcstatus` are now on-demand and formatted to match the upstream library `SettingsAndStatus` example output.
- `esp32s3_Anna`: AS5600 sampling rate reduced (UI/visualisation only), and the OLED UI no longer shows link status or driver fault bytes; `AP` and `RP` use the same font size and temperature is hidden unless valid.

## 2025-12-15
- `esp32s3_Anna`: OLED position display now shows `AP` (absolute degrees) plus `RP` (relative steps), and RP resets to 0 when the limit switch is hit (including held at boot). Status frames now populate `pos_steps` with the RP value.
- `teensy41_Beata`: status line `dist:` now prints the received `pos_steps` (RP) instead of the reserved `dist_mm` placeholder.

## 2025-12-06
- `esp32s3_Anna`: removed VL6180X support (telemetry keeps `dist:---`), simplified OLED UI to temperature/position/limit/driver, and added a STOP overlay + LED flash when the host issues a stop. Limit switch presses now block motion that would increase the angle, the driver idles on link loss, and the AS5600 loop runs at 1 kHz with an 800 kHz encoder bus. Standstill defaults to `normal`, hold current is reduced, and the TMC microstep setting is auto-corrected to the expected value with an event warning.
- `teensy41_Beata`: CLI trimmed to `move` and `moveto` (path/smooth/time planners removed); `maxvelocity` now defaults to and is capped at 100 sps, and status lines include a live `dps` field. Stops and limit trips also send a STOP alert to the axis UI, and the moveto planner uses a short 5% final window with a lower 40 dps cap.
- `lib/proto`: refactored `recvMessage` to decode buffered frames via a helper that always resets the buffer and validates CRC/length before copying payloads, improving robustness when framing errors occur.

## 2025-10-26
- `esp32s3_Anna` now checks for a DS18B20 probe before starting the temperature loop, reports its availability on boot, and keeps host telemetry at `NaN` when the probe is missing instead of recycling stale values.
- The OLED header hides temperature placeholders when the sensor is absent so the display reflects real data only.

## 2025-10-21
- Added persisted `standstillMode` handling on `esp32s3_Anna`; settings survive reboots and honor all TMC2209 standstill behaviours (`normal`, `freewheeling`, `braking`, `strong_braking`).
- Replaced the Teensy CLI `disablewhenstopped` toggle with `standstillMode <axis> ...` and ensured host stop/cancel logic keeps drivers enabled so standstill mode dictates coil state.
- Documented the Serial5 wiring for axis X1 and restored the firmware mapping to prevent regressions when auto-formatting the port table.
- Updated README and UART console docs to describe the new standstill workflow.

## 2025-10-20
- Per-axis max velocity overrides: `maxvelocity <axis> <sps>` sets a limit that applies only to that axis and persists in EEPROM; `maxvelocity <axis>` shows the axis' effective limit; `maxvelocity <sps>` still sets the global default.
- Motion planning and clamping updated to use the axis' effective `maxvelocity` everywhere (`move`, `moveto`).
- Console formatting unified: all axis-specific messages now start with the axis name (e.g., `R          -> moveto ...`, `Y          ! maxvelocity must be >0`).
- Telemetry line now shows `temp:---` when the DS18B20 temperature probe is absent/invalid (instead of `-127.0`).
- Added helper `consolePrintfAxis` and removed unused legacy clamp helper.

## 2025-10-18
- Split the AS5600 encoder onto a dedicated I2C bus and synchronized clock/timeouts for the shared peripherals on `esp32s3_Anna`.
- Trigger LED refreshes when the limit switch toggles and tighten debounce timing on `esp32s3_Anna`.
- Print link status twice as often in the `teensy41_Beata` loop to improve connectivity insight.
- Added a `setname` host command that labels each UART link (R/Y/X1/X2) and pushes the device name to the paired ESP32 display.
- Refactored Teensy `teensy41_Beata` CLI to axis-first syntax (e.g., `move y 100 100`), removed the `use` command, and updated `stop` to accept an optional axis name (`stop y`) in addition to stopping all axes when called without arguments.
