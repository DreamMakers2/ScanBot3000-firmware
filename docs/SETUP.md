# Firmware Setup

This guide covers the repository-supported PlatformIO build and the hardware interfaces explicitly documented by the project.

## 1. Install tooling

Install Git and PlatformIO Core, or install the PlatformIO extension in VS Code. The repository does not establish one tested host operating-system release, so no OS-specific minimum is claimed.

Verify PlatformIO:

```bash
pio --version
```

## 2. Clone

```bash
git clone https://github.com/DreamMakers2/ScanBot3000-firmware.git
cd ScanBot3000-firmware
```

Do not copy an existing developer `.pio` directory or editor state into the clone; dependencies should resolve from `platformio.ini`.

## 3. Build

ESP32-S3 axis controller:

```bash
pio run -e esp32s3_Anna
```

Teensy 4.1 supervisor:

```bash
pio run -e teensy41_Beata
```

The sanitized public configuration resolves the VL6180X driver from PlatformIO instead of relying on a machine-local library directory that was not present in the repository. Revalidate this dependency on the target hardware before treating the public release as a production firmware baseline.

## 4. Connect hardware

The verified project topology is:

- Teensy 4.1 supervisor.
- ESP32-S3 axis controllers for R, Z, X1, X2.
- Teensy Serial8 ↔ Raspberry Pi primary UART, common ground, 1,000,000 baud.
- Per-axis stepper/sensor wiring follows the pin definitions in `src/esp32s3_Anna/anna_pins.h`.

For the established Raspberry Pi mirror wiring, see `docs/pi_uart_console.md`.

## 5. Upload

Connect only the target board you intend to flash, then run:

```bash
pio run -e esp32s3_Anna -t upload
# or
pio run -e teensy41_Beata -t upload
```

If multiple serial devices are attached, use PlatformIO's normal upload-port selection rather than committing a machine-specific device path.

## 6. First console test

For USB monitoring:

```bash
pio device monitor -b 115200
```

Confirm that expected axis links appear and telemetry is plausible before enabling motion.

## 7. Safe motion validation

1. Verify each limit switch and motor direction with power/motion constrained.
2. Confirm driver settings and current before large moves.
3. Use small relative moves first.
4. Run the documented homing sequence before X/Z/P `moveabs` operations.
5. Keep physical stop access available during testing.

## 8. Integrate the control server

Follow the separate `ScanBot3000-control` setup guide for the Raspberry Pi FastAPI/UART bridge. Do not expose that unauthenticated motion-control service directly to an untrusted network.
