// lib/proto/proto.cpp
#include "proto.h"
#include <string.h>

// ---------- Constants ----------
static constexpr uint8_t  kMagic         = 0xA5;
static constexpr uint16_t kMaxPayload    = 128;     // fits Status/Command; raise if needed
static constexpr uint16_t kHdrSize       = 1 /*magic*/ + 1 /*type*/ + 2 /*len*/;
static constexpr uint16_t kCrcSize       = 2;
static constexpr uint16_t kMaxRaw        = kHdrSize + kMaxPayload + kCrcSize;
static constexpr uint16_t kMaxEncoded    = kMaxRaw + (kMaxRaw / 254) + 2; // COBS overhead + slack

static_assert(sizeof(StatusFrame)  == 23, "StatusFrame size unexpected");
static_assert(sizeof(CommandFrame) == 14, "CommandFrame size unexpected");
static_assert(sizeof(LedFrame)     == 32, "LedFrame size unexpected");

// ---------- CRC16-X25 (poly 0x1021, refin/refout, init 0xFFFF, xorout 0xFFFF) ----------
uint16_t crc16_x25(const uint8_t* d, uint16_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *d++;
    for (int i = 0; i < 8; ++i) {
      if (crc & 1) crc = (crc >> 1) ^ 0x8408;  // reversed 0x1021
      else         crc = (crc >> 1);
    }
  }
  return ~crc;
}

// ---------- COBS ----------
uint16_t cobs_encode(const uint8_t* in, uint16_t len, uint8_t* out) {
  uint16_t read_index = 0, write_index = 1, code_index = 0;
  uint8_t code = 1;

  while (read_index < len) {
    if (in[read_index] == 0) {
      out[code_index] = code;
      code_index = write_index++;
      code = 1;
      ++read_index;
    } else {
      out[write_index++] = in[read_index++];
      ++code;
      if (code == 0xFF) {
        out[code_index] = code;
        code_index = write_index++;
        code = 1;
      }
    }
  }
  out[code_index] = code;
  return write_index; // length of encoded data (no delimiter)
}

bool cobs_decode(const uint8_t* in, uint16_t len, uint8_t* out, uint16_t& outlen) {
  uint16_t read_index = 0, write_index = 0;
  while (read_index < len) {
    uint8_t code = in[read_index];
    if (code == 0 || (read_index + code) > (uint16_t)(len + 1)) return false;
    ++read_index;
    for (uint8_t i = 1; i < code; ++i) {
      if (read_index >= len) return false;
      out[write_index++] = in[read_index++];
    }
    if (code != 0xFF && read_index < len) {
      out[write_index++] = 0x00;
    }
  }
  outlen = write_index;
  return true;
}

// ---------- Internal helpers ----------
static void sendPacket(Stream& s, uint8_t type, const uint8_t* payload, uint16_t plen) {
  if (plen > kMaxPayload) return;  // guard, or replace with chunking if you raise kMaxPayload

  uint8_t raw[kMaxRaw];
  uint16_t idx = 0;

  // Header
  raw[idx++] = kMagic;
  raw[idx++] = type;
  raw[idx++] = (uint8_t)(plen & 0xFF);
  raw[idx++] = (uint8_t)((plen >> 8) & 0xFF);

  // Payload
  memcpy(&raw[idx], payload, plen);
  idx += plen;

  // CRC over header+payload
  const uint16_t crc = crc16_x25(raw, idx);
  raw[idx++] = (uint8_t)(crc & 0xFF);
  raw[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  // COBS encode + delimiter 0x00
  uint8_t enc[kMaxEncoded];
  uint16_t enc_len = cobs_encode(raw, idx, enc);
  s.write(enc, enc_len);
  s.write((uint8_t)0x00); // frame delimiter
}

void sendStatus(Stream& s, const StatusFrame& st) {
  sendPacket(s, MSG_STATUS, reinterpret_cast<const uint8_t*>(&st), sizeof(StatusFrame));
}

void sendCommand(Stream& s, const CommandFrame& cmd) {
  sendPacket(s, MSG_CMD, reinterpret_cast<const uint8_t*>(&cmd), sizeof(CommandFrame));
}

void sendCommandPayload(Stream& s, const uint8_t* payload, uint16_t len) {
  if (!payload || len == 0) return;
  sendPacket(s, MSG_CMD, payload, len);
}

void sendEvent(Stream& s, const uint8_t* data, uint16_t len) {
  if (!data || len == 0) return;
  if (len > kMaxPayload) len = kMaxPayload; // simple cap; caller can chunk
  sendPacket(s, MSG_EVENT, data, len);
}

// Persistent RX state per Stream (supports multiple UART ports).
struct RxState {
  uint8_t enc[kMaxEncoded];
  uint16_t enc_len = 0;
};

struct RxSlot {
  Stream*  stream = nullptr;
  RxState  state{};
  uint32_t last_use = 0;
};

// Keep independent reassembly state per Stream. This matters when the host polls multiple UARTs:
// a partial frame on one port must not be discarded just because another port was serviced.
static constexpr size_t kRxSlots = 8;
static RxSlot g_rx_slots[kRxSlots];
static uint32_t g_rx_use_counter = 0;

static RxState& rxStateFor(Stream& s) {
  Stream* sp = &s;
  size_t free_idx = kRxSlots;
  size_t lru_idx = 0;
  uint32_t lru_use = UINT32_MAX;

  for (size_t i = 0; i < kRxSlots; ++i) {
    RxSlot& slot = g_rx_slots[i];
    if (slot.stream == sp) {
      slot.last_use = ++g_rx_use_counter;
      return slot.state;
    }
    if (slot.stream == nullptr && free_idx == kRxSlots) {
      free_idx = i;
    }
    if (slot.last_use < lru_use) {
      lru_use = slot.last_use;
      lru_idx = i;
    }
  }

  size_t idx = (free_idx != kRxSlots) ? free_idx : lru_idx;
  g_rx_slots[idx].stream = sp;
  g_rx_slots[idx].state.enc_len = 0;
  g_rx_slots[idx].last_use = ++g_rx_use_counter;
  return g_rx_slots[idx].state;
}

// Decode the currently buffered frame (delimited by 0x00) into payload when valid.
// Returns true on success and resets the buffered state either way.
static bool decodeBufferedFrame(RxState& state, uint16_t cap, uint8_t& type, uint8_t* payload, uint16_t& len_out) {
  if (state.enc_len == 0) {
    state.enc_len = 0;
    return false;
  }

  uint8_t raw[kMaxRaw];
  uint16_t raw_len = 0;
  bool ok = cobs_decode(state.enc, state.enc_len, raw, raw_len);
  state.enc_len = 0; // reset for next frame regardless of result
  if (!ok) return false;

  if (raw_len < (kHdrSize + kCrcSize)) return false;

  // CRC check
  const uint16_t body_len = raw_len - kCrcSize;
  const uint16_t crc_calc = crc16_x25(raw, body_len);
  const uint16_t crc_rx   = (uint16_t)raw[body_len] | ((uint16_t)raw[body_len + 1] << 8);
  if (crc_calc != crc_rx) return false;

  // Header parse
  if (raw[0] != kMagic) return false;
  type = raw[1];
  const uint16_t plen = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
  if (kHdrSize + plen + kCrcSize != raw_len) return false;

  // Capacity check: caller passes capacity in 'cap'
  len_out = plen; // report needed/used length
  if (plen > cap) return false;

  memcpy(payload, &raw[kHdrSize], plen);
  return true;
}

bool recvMessage(Stream& s, uint8_t& type, uint8_t* payload, uint16_t& len /*IN=capacity, OUT=used*/) {
  RxState& state = rxStateFor(s);

  while (s.available()) {
    uint8_t b = (uint8_t)s.read();

    if (b == 0x00) {
      uint16_t cap = len;
      if (decodeBufferedFrame(state, cap, type, payload, len)) {
        return true;
      }
      continue;
    } else {
      // Accumulate encoded bytes (drop on overflow until delimiter arrives)
      if (state.enc_len < kMaxEncoded) {
        state.enc[state.enc_len++] = b;
      } else {
        // overflow: keep collecting until 0x00 then frame will be discarded by size checks
      }
    }
  }
  return false; // no complete frame yet
}
