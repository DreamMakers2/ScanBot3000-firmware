> For an overview of the firmware and CLI commands, see the root README.md.
> For coordinated-move workflows and safe testing steps, see `docs/pi_moveabs_ops.md`.

# Teensy41 UART Console For Raspberry Pi

## UART Parameters
- Interface: Teensy 4.1 `Serial8` port (RX8=pin 34, TX8=pin 35 on the underside)
- Logic level: 3.3 V TTL (directly compatible with Raspberry Pi UART)
- Baud rate: 1,000,000 bps
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow control: None (no RTS/CTS)
- Line termination: Teensy transmits `\n`; receiver may send `\n` or `\r\n` (carriage return is ignored)

## Console Mapping
- The UART mirrors the existing USB console output byte-for-byte.
- Any command accepted over USB can be issued over the Serial8 link without changes.
- Console responses, status updates, and log messages are written simultaneously to USB and Serial8.

## Data Direction
- Teensy -> Raspberry Pi: complete console output stream (startup banner, help text, status updates, command responses).
- Raspberry Pi -> Teensy: ASCII command lines identical to what would be typed on the USB console. Commands should be terminated with a newline.

## Performance Notes
- Serial8 runs at 1,000,000 bps; keep ground common and use short, shielded wiring for best signal integrity.
- Teensy 4.1 can service this link alongside up to six other UART ports, but validate for overruns if every link streams heavily.

## Console Command Reference
- Commands are case-insensitive and use an axis-first syntax. Provide the axis name first for any axis-targeted command (defaults are `R`, `Z`, `X1`, `X2` unless renamed via `setname`).
- When both X1 and X2 are connected, virtual axes `x` (0.5*(X1-X2)) and `p` (0.5*(X1+X2)) appear in status output and can be used with `move`.
- `driverstatus` and `driversettings` accept physical axes only (no virtual `x`/`p`).

- `help` or `?`
  - Print the full command list and usage summary.
- `en <axis>`
  - Enable the specified axis driver (command ID 1).
- `dis <axis>`
  - Disable the specified axis driver (command ID 0).
- `driverstatus <axis>`
  - Request the TMC2209 driver status report from the specified axis board.
- `driversettings <axis> [enable|disable]`
  - Request the TMC2209 driver settings report, or toggle the driver enable state.
- `cur <axis> <0-100>`
  - Set the driver run current percentage; values are clamped to 0..100 before sending.
- `ms <axis> <steps>`
  - Set the microstepping configuration (e.g. 8, 16, 32). Accepts any integer supported by the driver.
- `move <axis> <steps> <velocity> [acceleration]`
  - Open-loop relative move by step count using AccelStepper (no encoder feedback). `steps` can be an integer, `?` (approach limit), or `-?` (release). `x` drives X1+/X2- and `p` drives X1+/X2+. Units are full steps (scaled by microsteps).
- `moveto <axis> <deg>`
  - Drive to an absolute angle in degrees using a fast approach: cruise at `maxvelocity`, then a short final window with a damped approach to avoid overshoot. Completes when within ~1–2° and slow.
- `pos`
  - Print current positions for X/Z/P/R plus the `homed` flag in one line.
- `moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]`
  - Coordinated absolute move in steps for any subset of axes. Unspecified axes hold position; queued by default if another move is running. Requires homing for X/Z/P targets.
- `home z`
  - Request the host-side homing sequence (requires the Z limit switch to be pressed).
- `coordstatus`
  - Show coordinated-move state (state, homed flag, queue depth, last error, active axes).
- `maxvelocity [sps]` or `maxvelocity <axis> [sps]`
  - Without axis: get or set the global default speed cap (steps/s), capped at 100 sps.
  - With axis: query or set a per-axis override that affects only that axis. Overrides persist in EEPROM and are capped at 100 sps; omit `[sps]` to show the axis’ effective value.
- `maxaccel [sps^2]` or `maxaccel <axis> [sps^2]`
  - Get or set the global or per-axis acceleration cap. Higher values shorten ramps and reduce early braking.
- `showconfig`
  - Print the current motion caps (max velocity/accel).
- `standstillMode <axis> <normal|freewheeling|braking|strong_braking>`
  - Set the TMC2209 standstill behaviour for the given axis. The choice persists on the axis and defaults to `normal` (holding torque).
- `stop [axis]`
  - Stop all axes, or just the specified axis when given. Also aborts active jogs/homing and clears the coordinated-move queue.
- `tmcsettings <axis>`
  - Request the TMC2209 driver configuration report from the specified axis board.
- `tmcstatus <axis>`
  - Request the TMC2209 driver status bits from the specified axis board.
- `led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]`
  - Update all 8 LEDs in one frame. Each `<ledN>` token is either `RRGGBB` (6 hex chars, RGB order) or `------` to leave that LED unchanged. `000000` explicitly turns an LED off.
  - LED0 is reserved for link status; if no LED command is received for 5 seconds the axis restores LED0 to link status.
  - `T=<ms>` sets an optional fade duration; omit for immediate apply. `B=<0-255>` sets the global brightness used for subsequent LED updates (brightness changes are faded when `T` is supplied). Optional arguments can appear in any order after the 8 color tokens.
- `measure <axis> <seconds>`
  - Sample VL6180X range at 50 Hz for the given duration (axis R only). Each sample emits a `range_mm:<value>` line on the console; errors emit `range.err=<code>` (throttled, VL6180X status codes). These arrive as axis-prefixed event lines (for example, `R       range_mm:123.0`).
- `hi <axis>`
  - Instruct the axis to display "Hello" on the attached OLED.
- `reboot <axis>`
  - Reboot the specified axis controller.

`maxvelocity` and `maxaccel` accept the virtual axis tokens `x` and `p` to set caps used by coordinated `moveabs` planning. When both X and P are requested together, the more restrictive cap is applied.

Homing is requested from the Z-axis controller home button (GPIO1), or via `home z`; the Teensy runs the multi-axis sequence when the Z limit switch is already pressed. Use `stop` to abort.

## Example Session
```
Teensy41 link ready @1Mbaud on Serial8
Commands:
  en <axis>             -> enable driver
  dis <axis>            -> disable driver
  ...
Z   ts:123456 ang:  45.0 dps:0.00 dist:0 temp:32.5 lim:0 drv:ON flt:0x00

# Raspberry Pi sends over UART:
moveto z 180

-> moveto start Z -> 180.00deg
Z   ts:123956 ang:  60.0 dps:80.0 dist:120 temp:32.6 lim:0 drv:ON flt:0x00 tgt:180.00 err:+120.00
...
-> plan complete (moveto, 1.750s)
```
- The Teensy immediately prints the same output on USB and the Pi UART.
- The Pi issues commands by writing the ASCII line followed by `\n`.
- The `dist` field in the status line reports the axis relative position (`RP`) in steps since boot (reset to 0 when the axis limit switch is hit).
```
On Raspberry Pi:
$ stty -F /dev/ttyAMA0 1000000 cs8 -cstopb -parenb -ixon -ixoff
$ cat /dev/ttyAMA0  # view console output
$ echo -e "moveto z 180\n" > /dev/ttyAMA0
```
- The Pi must keep ground common with the Teensy. No additional framing or checksums are required.
