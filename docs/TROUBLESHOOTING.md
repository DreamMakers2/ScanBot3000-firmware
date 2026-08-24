# Firmware Troubleshooting

Only issues supported by current behavior or repository history are listed here.

## VL6180X range sensor is unreliable or absent

Project history records a change from an 800 kHz to a 400 kHz I2C bus to address the distance sensor. Verify the current bus configuration, short I2C wiring, common ground, sensor power, and the R-axis pin map before changing timing. The current measurement loop schedules reads every 20 ms.

The public-ready PlatformIO file also replaces a missing machine-local VL6180X library path with the registry package. If a build succeeds but sensor behavior changes, compare library behavior on the actual R-axis hardware before tuning the firmware.

## DS18B20 temperature shows `---`

The current firmware treats a missing/invalid probe as unavailable. It retries discovery after boot rather than inventing a value. Check the one-wire wiring and probe before changing telemetry parsing.

## Encoder angle is missing

Non-R axes expect an AS5600. When the encoder is absent, the firmware deliberately suppresses the associated angle-position display. Verify I2C presence and axis identity first.

## Axis link drops

Project history includes link-timeout tuning. Confirm both ends are configured for the expected 1,000,000 baud link and that wiring/ground integrity is good before increasing timeouts.

## Coordinated move is rejected

Use `coordstatus` and check the reported error. X/Z/P coordinated targets require a valid homed state and must remain within the firmware's configured soft limits. A link disconnect or unexpected limit event invalidates/aborts coordinated movement.

## Build cannot find a library

Delete local `.pio` build/dependency state and rerun PlatformIO so dependencies resolve from `platformio.ini`. Do not copy another developer's `.pio` directory into the repository.
