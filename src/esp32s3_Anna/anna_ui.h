#pragma once

#include <stddef.h>
#include <stdint.h>

#include "proto.h"

// OLED + NeoPixel UI (single owner of display + LED writes).
void annaUiInit();

// Thread-safe UI updates (wake the UI task).
void annaUiNotify();

void annaUiSetDeviceName(const char* name, size_t len);
void annaUiShowMessage(const char* msg, uint32_t duration_ms);
void annaUiTriggerStopAlert();
void annaUiSetPixelOverride(uint8_t idx, uint32_t rgb, uint8_t brightness);
void annaUiSetLedFrame(const LedFrame& frame);
void annaUiConfigureAxisR();
bool annaUiRequestMeasure(float duration_s);

// FreeRTOS task entry point.
void annaUiTask(void*);
