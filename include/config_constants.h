#pragma once

#include <stddef.h>
#include <stdint.h>

// Shared configuration knobs for host-side firmware.

// Global max velocity ceiling in steps/s; applies to all axes unless overridden per-axis.
static constexpr float kMaxVelocityCeiling = 1000.0f;

// Per-axis fixed mapping signs: +1 normal, -1 inverted, 0 = unknown.
// Index order matches links[] (R, Z, X1, X2); Z inversion is the hardware default.
static constexpr int8_t kFixedMappingSigns[] = { 0, -1, 0, 0 };
static constexpr size_t kFixedMappingSignsCount = sizeof(kFixedMappingSigns) / sizeof(kFixedMappingSigns[0]);
