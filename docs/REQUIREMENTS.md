# Firmware Requirements

This document separates hardware/software that is directly evidenced by the repository from requirements that are unknown. It intentionally avoids invented minimums.

## Verified project hardware configuration

### Motion supervisor

- **Board:** Teensy 4.1.
- **Role:** motion supervisor, coordinated-motion/homing logic, USB console, axis aggregation, Raspberry Pi UART mirror.
- **Host link:** Serial8 to Raspberry Pi at 1,000,000 baud in the documented configuration.

### Axis controllers

- **Board target:** `esp32-s3-devkitc-1` in PlatformIO.
- **Flash configuration:** 16 MB declared by `platformio.ini`.
- **PSRAM:** enabled by `platformio.ini`.
- **Stepper driver:** TMC2209.
- **Local display/indication:** SSD1306 OLED and WS2812B/NeoPixel output.
- **Temperature:** DS18B20.
- **Limit input:** digital limit switch.
- **Non-R position sensing:** AS5600 magnetic encoder.
- **R-axis ranging:** VL6180X time-of-flight sensor; the R-axis code repurposes the encoder-side path for ranging/activity indication.

### Logical machine layout

The firmware's established default labels are `R`, `Z`, `X1`, and `X2`. The Teensy also exposes virtual `X` and `P` coordinates derived from X1/X2 motion.

## Minimum hardware requirements

No generalized minimum CPU, RAM, flash, motor, power-supply, or sensor substitute has been validated by repository evidence. The supported baseline is therefore the board/sensor configuration above rather than a claimed lower minimum.

## Recommended hardware

No alternate or higher-spec recommended controller hardware has been verified in this repository. Use the documented boards for reproducibility unless you are prepared to port and retest pin assignments, timing, and libraries.

## Host development software

- Git.
- PlatformIO Core or PlatformIO IDE.
- Arduino framework through PlatformIO.

### PlatformIO platform configuration

- ESP32-S3 environment: `espressif32 @ ^6.5.0`, board `esp32-s3-devkitc-1`.
- Teensy environment: PlatformIO `teensy` platform, board `teensy41`.

### Declared ESP32-S3 libraries

The current `platformio.ini` declares Adafruit NeoPixel, Bounce2, AccelStepper `^1.64`, AS5600 `^0.6.6`, Adafruit GFX `^1.12.1`, DallasTemperature `^4.0.4`, Adafruit SSD1306 `^2.5.15`, TMC2209 `^10.1.0`, Adafruit BusIO `^1.17.4`, Adafruit VL6180X, and OneWire `^2.3.8`, plus Arduino Wire/SPI.

The sanitized public release replaces a previously missing machine-local VL6180X library path with the official PlatformIO registry dependency. The exact historical local library revision cannot be verified and no equivalence claim is made until hardware revalidation.

## Connected-system requirements

The full ScanBot3000 stack additionally uses a Raspberry Pi 4B control host and a browser-based kinematics client; those requirements are maintained in their respective repositories and summarized in the project-home requirements document.
