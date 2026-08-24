#pragma once

#include <Arduino.h>

#include "proto.h"

using AnnaCommandCallback = void (*)(const CommandFrame& cmd, uint32_t now_ms);
using AnnaLedCallback = void (*)(const LedFrame& frame, uint32_t now_ms);

// Serial1 link to Beata (Teensy).
void annaLinkInit();

// Poll RX and invoke callback for each decoded CommandFrame.
void annaLinkPollRx(uint32_t now_ms, AnnaCommandCallback on_cmd, AnnaLedCallback on_led);

// Thread-safe framed TX helpers (COBS+CRC).
void annaLinkSendStatus(const StatusFrame& st);
void annaLinkSendCommand(const CommandFrame& cmd);
void annaLinkSendEvent(const uint8_t* data, uint16_t len);
void annaLinkSendEventText(const char* text);
