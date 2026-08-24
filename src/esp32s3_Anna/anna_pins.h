#pragma once

#include <stdint.h>

// Pin map (ESP32-S3)
static constexpr int PIN_TMC_EN    = 21;
static constexpr int PIN_TMC_MS1   = 5;
static constexpr int PIN_TMC_MS2   = 6;
static constexpr int PIN_TMC_RX    = 18; // MCU TX (GPIO18) -> BTT TMC2209 PDN/UART via 1k
static constexpr int PIN_TMC_TX    = 17; // MCU RX (GPIO17) <- BTT TMC2209 PDN/UART direct
static constexpr int PIN_TMC_STEP  = 9;
static constexpr int PIN_TMC_DIR   = 10;
static constexpr int PIN_TMC_INDEX = 15;
static constexpr int PIN_TMC_DIAG  = 11;

static constexpr int PIN_ONEWIRE   = 16;
static constexpr int PIN_LIMIT     = 2;
static constexpr int PIN_HOMING_BTN = 1;
static constexpr int PIN_NEOPIX    = 41;
static constexpr int PIN_AS5600_DIR = 37;

static constexpr int I2C_SHARED_SDA_PIN  = 4;
static constexpr int I2C_SHARED_SCL_PIN  = 7;
static constexpr int I2C_ENCODER_SDA_PIN = 12;
static constexpr int I2C_ENCODER_SCL_PIN = 13;

// Axis R only: repurpose encoder I2C pins after axis identity is confirmed.
static constexpr int PIN_R_VL6180X_SHUT = 12;
static constexpr int PIN_R_NEOPIX       = 13;

static constexpr uint32_t I2C_SHARED_CLOCK_HZ  = 400000; // 400 kHz for SSD1306
static constexpr uint32_t I2C_ENCODER_CLOCK_HZ = 800000; // fast bus shortens exposure to stepper EMI (board has strong pull-ups)

static constexpr uint16_t kNeoPixelCount = 8;
