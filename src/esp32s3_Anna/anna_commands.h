#pragma once

#include <stdint.h>

#include "proto.h"

// Dispatch a decoded CommandFrame from Beata.
void annaHandleCommand(const CommandFrame& cmd, uint32_t now_ms);
void annaHandleLedFrame(const LedFrame& frame, uint32_t now_ms);
