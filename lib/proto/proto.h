#pragma once
#include <Arduino.h>   // gives you Stream, Print, etc.
#include <stdint.h>

#include "protocol_constants.h"

// Message types
enum : uint8_t { MSG_CMD=1, MSG_STATUS=2, MSG_ACK=3, MSG_NACK=4, MSG_EVENT=5 };

struct __attribute__((packed)) StatusFrame {
  uint32_t ts_us;
  int32_t  pos_steps;
  float    angle_deg;
  float    dist_mm;   // reserved for downstream parser; keep even when unused
  float    temp_c;
  uint8_t  limit, fault, driver; // driver: 1=enabled, 0=disabled
};

struct __attribute__((packed)) CommandFrame {
  uint8_t axis_id, cmd;     // cmd uses CommandId (see protocol_constants.h); axis_id is a command-specific extra byte.
  float   p0, p1, p2;
};

struct __attribute__((packed)) LedFrame {
  uint8_t  cmd; // CommandId::SetPixels
  uint8_t  flags;
  uint8_t  brightness;
  uint8_t  mask; // bit i: update LED i
  uint32_t transition_ms;
  uint8_t  rgb[kLedFrameLedCount][3]; // R,G,B
};

// COBS + CRC16-X25 (same code compiled on both sides)
uint16_t crc16_x25(const uint8_t* d, uint16_t n);
uint16_t cobs_encode(const uint8_t* in, uint16_t len, uint8_t* out);
bool     cobs_decode(const uint8_t* in, uint16_t len, uint8_t* out, uint16_t& outlen);

// Convenience
void sendStatus(Stream& s, const StatusFrame& st);
void sendCommand(Stream& s, const CommandFrame& cmd);
void sendCommandPayload(Stream& s, const uint8_t* payload, uint16_t len);
bool recvMessage(Stream& s, uint8_t& type, uint8_t* payload, uint16_t& len);
void sendEvent(Stream& s, const uint8_t* data, uint16_t len);
