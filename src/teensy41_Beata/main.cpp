#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <EEPROM.h>
#include "proto.h"
#include "config_constants.h"
#include "mirror_console.h"

#ifdef __INTELLISENSE__
// INTELLISENSE shim: keep list in sync with actual link ports. X1 must stay on Serial5.
extern HardwareSerial Serial8;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
#endif

static constexpr size_t kLinkNameMaxLen = 8;
static constexpr size_t kMaxPersistLinks = 8;

struct Config {
  float    max_velocity;
  float    max_accel;
  uint32_t reserved0;
  uint32_t reserved1;
};

static constexpr Config kConfigDefaults{ kMaxVelocityCeiling, 500.0f, 0u, 0u };

// Unified persistence blocks. Keep them small: config + per-axis caps.
static constexpr uint32_t kPersistMagicV3 = 0x53425033u; // 'SBP3'
static constexpr uint16_t kPersistVersionV3 = 3;
static constexpr uint32_t kPersistMagicV4 = 0x53425034u; // 'SBP4'
static constexpr uint16_t kPersistVersionV4 = 4;

struct PersistV3 {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  Config   cfg;
  float    axis_maxvel[kMaxPersistLinks];
  uint8_t  axis_maxvel_valid[kMaxPersistLinks];
};

struct PersistV4 {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  Config   cfg;
  float    axis_maxvel[kMaxPersistLinks];
  uint8_t  axis_maxvel_valid[kMaxPersistLinks];
  float    axis_maxaccel[kMaxPersistLinks];
  uint8_t  axis_maxaccel_valid[kMaxPersistLinks];
  float    virtual_maxvel[2];     // X, P
  uint8_t  virtual_maxvel_valid[2];
  float    virtual_maxaccel[2];   // X, P
  uint8_t  virtual_maxaccel_valid[2];
};

static PersistV4 g_persist{};
static bool g_persist_ok = false;

enum class MotionMode : uint8_t {
  Idle,
  MoveTo,
};

static constexpr uint32_t kVelocityMinDeltaUs = 100u;      // ignore duplicate samples closer than 0.1 ms
static constexpr uint32_t kVelocityMaxDeltaUs = 1000000u;  // treat gaps >1 s as stale
static constexpr float    kVelocityFilterAlpha = 0.25f;
static constexpr float    kDirectionCheckMinDps = 80.0f;   // min |deg/s| to trust sign for auto-correct
static constexpr float    kMinSpsDeltaToUpdate = 1.0f;       // avoid spamming tiny changes
static constexpr float    kStepsPerDegMinEstimate = 0.2f;
static constexpr float    kStepsPerDegMaxEstimate = 5000.0f;
static constexpr float    kDefaultStepsPerDegEstimate = 50.0f;

// Soft limits for homed coordinates (full steps).
static constexpr int32_t  kSoftLimitZMin = -11500;
static constexpr int32_t  kSoftLimitZMax = -50;
static constexpr int32_t  kSoftLimitXMin = 0;
static constexpr int32_t  kSoftLimitXMax = 2100;
static constexpr int32_t  kSoftLimitPMin = -255;
static constexpr int32_t  kSoftLimitPMax = 255;

// Fast approach tuning: keep high speed until a very short final window,
// then drop to a fixed slow approach speed for accuracy without oscillation.
static constexpr float    kFinalWindowFraction = 0.05f;        // last X% of initial error
static constexpr float    kFinalWindowMinDeg   = 6.0f;         // but at least this many degrees
static constexpr float    kMoveToToleranceDeg  = 1.5f;         // "close enough" completion tolerance for moveto
static constexpr float    kFinalMaxDps         = 40.0f;        // cap final approach deg/s to limit crossing speed
static constexpr float    kFinalKpDpsPerDeg    = 8.0f;         // proportional final approach gain (deg/s per deg)

struct VelocityEstimator {
  float    dps = 0.0f;
  bool     valid = false;
  float    last_angle_deg = NAN;
  uint32_t last_ts_us = 0;
  bool     have_last = false;

  void reset() {
    dps = 0.0f;
    valid = false;
    last_angle_deg = NAN;
    last_ts_us = 0;
    have_last = false;
  }

  void update(const StatusFrame& st) {
    const uint32_t sample_ts = st.ts_us;
    const float sample_angle = st.angle_deg;
    if (!isfinite(sample_angle)) {
      reset();
      return;
    }

    if (!have_last) {
      have_last = true;
      last_ts_us = sample_ts;
      last_angle_deg = sample_angle;
      valid = false;
      dps = 0.0f;
      return;
    }

    const uint32_t dt_us = sample_ts - last_ts_us;
    const float da = sample_angle - last_angle_deg;
    last_ts_us = sample_ts;
    last_angle_deg = sample_angle;

    if (dt_us < kVelocityMinDeltaUs || dt_us > kVelocityMaxDeltaUs) {
      valid = false;
      dps = 0.0f;
      return;
    }

    const float dt_s = (float)dt_us * 1e-6f;
    const float inst_dps = da / dt_s;
    if (!isfinite(inst_dps)) {
      valid = false;
      dps = 0.0f;
      return;
    }

    if (!valid) {
      dps = inst_dps;
    } else {
      dps = kVelocityFilterAlpha * inst_dps + (1.0f - kVelocityFilterAlpha) * dps;
    }
    valid = true;
  }
};

struct MotionState {
  MotionMode mode = MotionMode::Idle;
  uint32_t   start_ms = 0;
  float      commanded_velocity_sps = 0.0f;
  float      cruise_velocity_sps = 0.0f; // magnitude, >= 0
  float      target_deg = 0.0f;
  float      slow_window_deg = 0.0f;
  float      tolerance_deg = 0.0f;
  bool       slowed = false;
  uint32_t   last_correction_ms = 0;
  float      start_error_abs = 0.0f;
};

struct SyntheticSteps {
  double   steps = 0.0;
  float    velocity_sps = 0.0f;
  uint32_t last_us = 0;
  bool     have_time = false;

  void reset() {
    steps = 0.0;
    velocity_sps = 0.0f;
    last_us = 0;
    have_time = false;
  }

  void accumulate() {
    uint32_t now = micros();
    if (!have_time) {
      have_time = true;
      last_us = now;
      return;
    }
    uint32_t delta = now - last_us;
    if (delta == 0) return;
    steps += static_cast<double>(velocity_sps) * (static_cast<double>(delta) / 1e6);
    last_us = now;
  }

  void setVelocity(float sps) {
    accumulate();
    velocity_sps = sps;
  }

  int32_t positionStepsSaturated() const {
    double v = steps;
    if (v > static_cast<double>(INT32_MAX)) v = static_cast<double>(INT32_MAX);
    if (v < static_cast<double>(INT32_MIN)) v = static_cast<double>(INT32_MIN);
    return static_cast<int32_t>(lround(v));
  }
};

struct Link {
  char             name[kLinkNameMaxLen + 1];
  HardwareSerial*  port;
  StatusFrame      last;
  uint32_t         last_ms;
  bool             was_connected;
  MotionState      motion;
  SyntheticSteps   synth;
  VelocityEstimator velocity;
};

static float estimateStepsPerDeg(const Link& link) {
  float ratio = NAN;
  if (link.velocity.valid && fabsf(link.velocity.dps) >= kDirectionCheckMinDps) {
    float cmd = fabsf(link.motion.commanded_velocity_sps);
    float dps = fabsf(link.velocity.dps);
    if (cmd > 1e-3f && dps > 1e-3f) {
      ratio = cmd / dps;
    }
  }
  if (!isfinite(ratio) || ratio <= 0.0f) {
    ratio = kDefaultStepsPerDegEstimate;
  }
  return constrain(ratio, kStepsPerDegMinEstimate, kStepsPerDegMaxEstimate);
}

static Config g_config = kConfigDefaults;

static constexpr uint8_t kStopButtonPin = 14;
static constexpr uint32_t kStopButtonDebounceMs = 50u;
static bool g_stop_button_prev_pressed = false;
static uint32_t g_stop_button_last_trigger_ms = 0;

struct VirtualAxesState {
  float x_offset_steps = 0.0f;
  float p_offset_steps = 0.0f;
};

static VirtualAxesState g_virtual_axes{};

enum class VirtualAxis : uint8_t {
  X = 0,
  P = 1,
};

enum class JogAxisKind : uint8_t {
  Physical,
  VirtualX,
  VirtualP,
};

struct JogUntilLimitState {
  bool        active = false;
  JogAxisKind kind = JogAxisKind::Physical;
  size_t      monitor_idx = 0;
  size_t      axis_a = 0;
  size_t      axis_b = 0;
  bool        dual = false;
  int8_t      sign = 0;
  float       vel_sps = 0.0f;
  uint8_t     start_limit = 0;
  uint32_t    start_ms = 0;
  uint32_t    timeout_ms = 60000;
};

static JogUntilLimitState g_jog_until_limit{};

enum class CoordAxis : uint8_t {
  X = 0,
  Z = 1,
  P = 2,
  R = 3,
  Count = 4,
};

static constexpr size_t kCoordQueueMax = 8;
static constexpr uint32_t kCoordTimeoutSlackMs = 500u;
static constexpr float kCoordTimeoutFactor = 1.25f;

struct CoordRequest {
  bool     has[(size_t)CoordAxis::Count] = {};
  int32_t  target[(size_t)CoordAxis::Count] = {};
};

struct CoordState {
  bool     active = false;
  CoordRequest request{};
  bool     axis_used[kMaxPersistLinks] = {};
  bool     axis_done[kMaxPersistLinks] = {};
  int32_t  axis_delta[kMaxPersistLinks] = {};
  float    axis_vel[kMaxPersistLinks] = {};
  float    axis_accel[kMaxPersistLinks] = {};
  uint32_t start_ms = 0;
  uint32_t expected_ms = 0;
  char     last_error[32] = {};
};

static CoordState g_coord{};
static CoordRequest g_coord_queue[kCoordQueueMax];
static size_t g_coord_queue_head = 0;
static size_t g_coord_queue_count = 0;

enum class HomeSeqPhase : uint8_t {
  Idle,
  ZRelease,
  ZApproach,
  ZToTop,
  PRelease,
  PApproach,
  XRelease,
  XApproach,
  CenterP,
  CenterX,
  CenterZ,
};

struct HomeSeqState {
  bool         request_pending = false;
  bool         active = false;
  HomeSeqPhase phase = HomeSeqPhase::Idle;
  bool         action_started = false;
  uint32_t     phase_start_ms = 0;

  bool     fixed_dual = false;
  size_t   fixed_a = 0;
  size_t   fixed_b = 0;
  int32_t  fixed_target_a = 0;
  int32_t  fixed_target_b = 0;
  uint32_t fixed_stable_since_ms = 0;
};

static HomeSeqState g_home{};
static bool g_soft_limits_enabled = false;

// Runtime per-axis max velocity overrides (fallback to global when not set)
static float g_axis_max_velocity[kMaxPersistLinks] = {};
static bool  g_axis_max_velocity_valid[kMaxPersistLinks] = {};
// Runtime per-axis max acceleration overrides (fallback to global when not set)
static float g_axis_max_accel[kMaxPersistLinks] = {};
static bool  g_axis_max_accel_valid[kMaxPersistLinks] = {};

// Runtime virtual-axis limits (X, P) for coordinated moves.
static float g_virtual_max_velocity[2] = {};
static bool  g_virtual_max_velocity_valid[2] = {};
static float g_virtual_max_accel[2] = {};
static bool  g_virtual_max_accel_valid[2] = {};

static Link links[] = {
  { "R",  &Serial1 },
  { "Z",  &Serial2 },
  // X1 link is wired to Serial5 (pins 47/48). Do not revert to Serial3; TX/RX3 is unused on this rev.
  { "X1", &Serial5 },
  { "X2", &Serial4 },
};
static constexpr size_t kNumLinks = sizeof(links) / sizeof(links[0]);
static constexpr size_t kIdxR = 0;
static constexpr size_t kIdxZ = 1;
static constexpr size_t kIdxX1 = 2;
static constexpr size_t kIdxX2 = 3;

static bool g_axis_move_active[kNumLinks] = {};
static uint32_t g_axis_move_start_ms[kNumLinks] = {};

static const char* kDefaultLinkNames[kNumLinks] = { "R", "Z", "X1", "X2" };
static HardwareSerial& piConsole = Serial8;
static MirrorConsole console(Serial, piConsole);

static inline bool axisBusy(size_t idx) {
  return links[idx].motion.mode != MotionMode::Idle;
}

static bool isLinkConnected(const Link& link, uint32_t now_ms);
static void stopJogUntilLimit(const char* reason);
static int32_t roundStepsSaturated(float v);
static void coordOnStop(const char* code);
static void markMoveStepsStarted(size_t idx);
static void markMoveStepsDone(size_t idx);

template <typename T>
static void consolePrint(const T& value) {
  console.print(value);
}

static void consolePrintln() {
  console.println();
}

template <typename T>
static void consolePrintln(const T& value) {
  console.println(value);
}

static void consoleWrite(uint8_t value) {
  console.write(value);
}

static void consolePrintf(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  console.vprintf(fmt, args);
  va_end(args);
}

// Print with axis prefix at start of line (padded to 8 chars), then formatted message
static void consolePrintfAxis(const char* axis_name, const char* fmt, ...) {
  if (!axis_name) axis_name = "";
  console.printf("%-8s ", axis_name);
  va_list args;
  va_start(args, fmt);
  console.vprintf(fmt, args);
  va_end(args);
}

static bool equalsIgnoreCase(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    unsigned char ca = static_cast<unsigned char>(*a++);
    unsigned char cb = static_cast<unsigned char>(*b++);
    if (tolower(ca) != tolower(cb)) return false;
  }
  return (*a == '\0' && *b == '\0');
}

static constexpr uint8_t kStandstillModeNormal = 0;
static constexpr uint8_t kStandstillModeFreewheeling = 1;
static constexpr uint8_t kStandstillModeStrongBraking = 2;
static constexpr uint8_t kStandstillModeBraking = 3;
static const char* kStandstillModeUsage = "Usage: standstillMode <axis> <normal|freewheeling|braking|strong_braking>";

static const char* standstillModeName(uint8_t mode) {
  switch (mode) {
    case kStandstillModeNormal:         return "normal";
    case kStandstillModeFreewheeling:   return "freewheeling";
    case kStandstillModeBraking:        return "braking";
    case kStandstillModeStrongBraking:  return "strong_braking";
    default:                            return "freewheeling";
  }
}

static bool parseStandstillModeToken(const char* token, uint8_t& out_mode) {
  if (!token) return false;
  if (equalsIgnoreCase(token, "normal")) {
    out_mode = kStandstillModeNormal;
    return true;
  }
  if (equalsIgnoreCase(token, "freewheeling")) {
    out_mode = kStandstillModeFreewheeling;
    return true;
  }
  if (equalsIgnoreCase(token, "braking")) {
    out_mode = kStandstillModeBraking;
    return true;
  }
  if (equalsIgnoreCase(token, "strong_braking") || equalsIgnoreCase(token, "strong-braking")) {
    out_mode = kStandstillModeStrongBraking;
    return true;
  }
  return false;
}

static inline void sendCmdTo(size_t idx, uint8_t cmd, float p0 = 0.0f, float p1 = 0.0f, float p2 = 0.0f) {
  CommandFrame c{};
  c.axis_id = 0;
  c.cmd     = cmd;
  c.p0      = p0;
  c.p1      = p1;
  c.p2      = p2;
  sendCommand(*links[idx].port, c);
}

static inline void sendVelocity(size_t idx, float sps) {
  links[idx].synth.setVelocity(sps);
  sendCmdTo(idx, (uint8_t)CommandId::MoveStepsPerSecond, sps, 0.0f);
}

static inline void sendStopAlert(size_t idx) {
  sendCmdTo(idx, (uint8_t)CommandId::StopAlert, 0.0f, 0.0f);
}

// (removed) sendCmd(): legacy active-axis helper; replaced by sendCmdTo(idx,...)

// Find link index by axis name (case-insensitive exact match)
static bool findLinkIndexByName(const char* axis_token, size_t& out_idx) {
  if (!axis_token) return false;
  for (size_t i = 0; i < kNumLinks; ++i) {
    if (equalsIgnoreCase(axis_token, links[i].name)) { out_idx = i; return true; }
  }
  return false;
}

enum class AxisTokenKind : uint8_t {
  None,
  Physical,
  VirtualX,
  VirtualP,
};

static bool parseAxisToken(const char* token, AxisTokenKind& kind, size_t& out_idx) {
  kind = AxisTokenKind::None;
  out_idx = 0;
  if (!token) return false;
  if (equalsIgnoreCase(token, "x")) {
    kind = AxisTokenKind::VirtualX;
    return true;
  }
  if (equalsIgnoreCase(token, "p")) {
    kind = AxisTokenKind::VirtualP;
    return true;
  }
  if (findLinkIndexByName(token, out_idx)) {
    kind = AxisTokenKind::Physical;
    return true;
  }
  return false;
}

static size_t linkNameLen(const char* s) {
  size_t len = 0;
  if (!s) return 0;
  while (len < kLinkNameMaxLen && s[len]) ++len;
  return len;
}

static void pushLinkName(size_t idx) {
  if (idx >= kNumLinks) return;
  CommandFrame c{};
  c.cmd = (uint8_t)CommandId::SetName;
  char name_buf[kLinkNameMaxLen + 1] = {};
  strncpy(name_buf, links[idx].name, kLinkNameMaxLen);
  name_buf[kLinkNameMaxLen] = '\0';
  size_t len = linkNameLen(name_buf);
  if (len > 0 && name_buf[len - 1] == '\n') {
    name_buf[len - 1] = '\0';
    len = linkNameLen(name_buf);
  }
  c.axis_id = static_cast<uint8_t>(len);
  uint32_t chunk0 = 0;
  uint32_t chunk1 = 0;
  memcpy(&chunk0, name_buf, (len > 4) ? 4 : len);
  if (len > 4) {
    size_t rest = len - 4;
    if (rest > 4) rest = 4;
    memcpy(&chunk1, name_buf + 4, rest);
  }
  memcpy(&c.p0, &chunk0, sizeof(chunk0));
  memcpy(&c.p1, &chunk1, sizeof(chunk1));
  sendCommand(*links[idx].port, c);
}

static void setLinkName(size_t idx, const char* name) {
  if (idx >= kNumLinks || !name) return;
  char sanitized[kLinkNameMaxLen + 1] = {};
  strncpy(sanitized, name, kLinkNameMaxLen);
  sanitized[kLinkNameMaxLen] = '\0';
  for (size_t i = 0; i < kLinkNameMaxLen && sanitized[i]; ++i) {
    if (static_cast<uint8_t>(sanitized[i]) < 32) sanitized[i] = '_';
  }
  strncpy(links[idx].name, sanitized, sizeof(links[idx].name));
  links[idx].name[sizeof(links[idx].name) - 1] = '\0';
  pushLinkName(idx);
}

static char linebuf[128];
static size_t linepos = 0;
static char pi_linebuf[128];
static size_t pi_linepos = 0;

static PersistV4 makePersistDefaultsV4() {
  PersistV4 p{};
  p.magic = kPersistMagicV4;
  p.version = kPersistVersionV4;
  p.reserved = 0;
  p.cfg = kConfigDefaults;
  for (size_t i = 0; i < kMaxPersistLinks; ++i) {
    p.axis_maxvel[i] = 0.0f;
    p.axis_maxvel_valid[i] = 0;
    p.axis_maxaccel[i] = 0.0f;
    p.axis_maxaccel_valid[i] = 0;
  }
  for (size_t i = 0; i < 2; ++i) {
    p.virtual_maxvel[i] = 0.0f;
    p.virtual_maxvel_valid[i] = 0;
    p.virtual_maxaccel[i] = 0.0f;
    p.virtual_maxaccel_valid[i] = 0;
  }
  return p;
}

static void persistEnsure() {
  if (!g_persist_ok) {
    g_persist = makePersistDefaultsV4();
    g_persist_ok = true;
  }
}

static bool persistLoadV4() {
  PersistV4 p{};
  EEPROM.get(0, p);
  if (p.magic != kPersistMagicV4 || p.version != kPersistVersionV4) return false;
  g_persist = p;
  g_persist_ok = true;
  return true;
}

static bool persistLoadV3(PersistV3& out) {
  EEPROM.get(0, out);
  if (out.magic != kPersistMagicV3 || out.version != kPersistVersionV3) return false;
  return true;
}

static void persistSaveV4() {
  persistEnsure();
  g_persist.magic = kPersistMagicV4;
  g_persist.version = kPersistVersionV4;
  g_persist.reserved = 0;
  EEPROM.put(0, g_persist);
}

static void applyPersistToRuntime() {
  // Config
  g_config = g_persist.cfg;
  if (!isfinite(g_config.max_velocity) || g_config.max_velocity <= 0.0f) g_config.max_velocity = kConfigDefaults.max_velocity;
  if (!isfinite(g_config.max_accel) || g_config.max_accel <= 0.0f)       g_config.max_accel = kConfigDefaults.max_accel;
  g_config.reserved0 = 0;
  g_config.reserved1 = 0;
  if (g_config.max_velocity > kMaxVelocityCeiling) g_config.max_velocity = kMaxVelocityCeiling;

  // Per-axis max velocity overrides
  size_t n = (kNumLinks < kMaxPersistLinks) ? kNumLinks : kMaxPersistLinks;
  for (size_t i = 0; i < kMaxPersistLinks; ++i) {
    g_axis_max_velocity_valid[i] = false;
    g_axis_max_velocity[i] = 0.0f;
    g_axis_max_accel_valid[i] = false;
    g_axis_max_accel[i] = 0.0f;
  }
  for (size_t i = 0; i < n; ++i) {
    if (g_persist.axis_maxvel_valid[i] == 1 && isfinite(g_persist.axis_maxvel[i]) && g_persist.axis_maxvel[i] > 0.0f) {
      g_axis_max_velocity[i] = min(g_persist.axis_maxvel[i], kMaxVelocityCeiling);
      g_axis_max_velocity_valid[i] = true;
    }
    if (g_persist.axis_maxaccel_valid[i] == 1 && isfinite(g_persist.axis_maxaccel[i]) && g_persist.axis_maxaccel[i] > 0.0f) {
      g_axis_max_accel[i] = g_persist.axis_maxaccel[i];
      g_axis_max_accel_valid[i] = true;
    }
  }

  // Virtual-axis caps (X, P) for coordinated moves.
  for (size_t i = 0; i < 2; ++i) {
    g_virtual_max_velocity_valid[i] = false;
    g_virtual_max_velocity[i] = 0.0f;
    g_virtual_max_accel_valid[i] = false;
    g_virtual_max_accel[i] = 0.0f;
  }
  for (size_t i = 0; i < 2; ++i) {
    if (g_persist.virtual_maxvel_valid[i] == 1 && isfinite(g_persist.virtual_maxvel[i]) && g_persist.virtual_maxvel[i] > 0.0f) {
      g_virtual_max_velocity[i] = min(g_persist.virtual_maxvel[i], kMaxVelocityCeiling);
      g_virtual_max_velocity_valid[i] = true;
    }
    if (g_persist.virtual_maxaccel_valid[i] == 1 && isfinite(g_persist.virtual_maxaccel[i]) && g_persist.virtual_maxaccel[i] > 0.0f) {
      g_virtual_max_accel[i] = g_persist.virtual_maxaccel[i];
      g_virtual_max_accel_valid[i] = true;
    }
  }
}

static void refreshPersistFromRuntime() {
  persistEnsure();
  g_persist.cfg = g_config;
  for (size_t i = 0; i < kMaxPersistLinks; ++i) {
    g_persist.axis_maxvel[i] = 0.0f;
    g_persist.axis_maxvel_valid[i] = 0;
    g_persist.axis_maxaccel[i] = 0.0f;
    g_persist.axis_maxaccel_valid[i] = 0;
  }
  size_t n = (kNumLinks < kMaxPersistLinks) ? kNumLinks : kMaxPersistLinks;
  for (size_t i = 0; i < n; ++i) {
    if (i < kMaxPersistLinks && g_axis_max_velocity_valid[i] && isfinite(g_axis_max_velocity[i]) && g_axis_max_velocity[i] > 0.0f) {
      g_persist.axis_maxvel[i] = g_axis_max_velocity[i];
      g_persist.axis_maxvel_valid[i] = 1;
    }
    if (i < kMaxPersistLinks && g_axis_max_accel_valid[i] && isfinite(g_axis_max_accel[i]) && g_axis_max_accel[i] > 0.0f) {
      g_persist.axis_maxaccel[i] = g_axis_max_accel[i];
      g_persist.axis_maxaccel_valid[i] = 1;
    }
  }
  for (size_t i = 0; i < 2; ++i) {
    g_persist.virtual_maxvel[i] = 0.0f;
    g_persist.virtual_maxvel_valid[i] = 0;
    g_persist.virtual_maxaccel[i] = 0.0f;
    g_persist.virtual_maxaccel_valid[i] = 0;
  }
  for (size_t i = 0; i < 2; ++i) {
    if (g_virtual_max_velocity_valid[i] && isfinite(g_virtual_max_velocity[i]) && g_virtual_max_velocity[i] > 0.0f) {
      g_persist.virtual_maxvel[i] = g_virtual_max_velocity[i];
      g_persist.virtual_maxvel_valid[i] = 1;
    }
    if (g_virtual_max_accel_valid[i] && isfinite(g_virtual_max_accel[i]) && g_virtual_max_accel[i] > 0.0f) {
      g_persist.virtual_maxaccel[i] = g_virtual_max_accel[i];
      g_persist.virtual_maxaccel_valid[i] = 1;
    }
  }
}

static void saveConfig() {
  refreshPersistFromRuntime();
  persistSaveV4();
}

static void printConfig() {
  consolePrintf("maxvelocity: %.1f sps\n", min(g_config.max_velocity, kMaxVelocityCeiling));
  consolePrintf("maxaccel: %.1f sps^2\n", g_config.max_accel);
}

static float getAxisMaxVelocity(size_t idx) {
  if (idx < kMaxPersistLinks && g_axis_max_velocity_valid[idx]) {
    return min(g_axis_max_velocity[idx], kMaxVelocityCeiling);
  }
  return min(g_config.max_velocity, kMaxVelocityCeiling);
}

static float getAxisMaxAccel(size_t idx) {
  if (idx < kMaxPersistLinks && g_axis_max_accel_valid[idx]) {
    return g_axis_max_accel[idx];
  }
  return g_config.max_accel;
}

static float getVirtualAxisMaxVelocity(VirtualAxis axis) {
  size_t idx = static_cast<size_t>(axis);
  if (idx < 2 && g_virtual_max_velocity_valid[idx]) {
    return min(g_virtual_max_velocity[idx], kMaxVelocityCeiling);
  }
  return min(g_config.max_velocity, kMaxVelocityCeiling);
}

static float getVirtualAxisMaxAccel(VirtualAxis axis) {
  size_t idx = static_cast<size_t>(axis);
  if (idx < 2 && g_virtual_max_accel_valid[idx]) {
    return g_virtual_max_accel[idx];
  }
  return g_config.max_accel;
}

static void saveAxisVelocityFor(size_t idx) {
  if (idx >= kNumLinks) return;
  refreshPersistFromRuntime();
  persistSaveV4();
}

static float clampVelocityLimit(float sps, float limit, bool& clamped) {
  clamped = false;
  if (limit <= 0.0f) return sps;
  if (fabsf(sps) > limit) {
    clamped = true;
    sps = copysignf(limit, sps);
  }
  return sps;
}

static float clampVelocityAxis(size_t idx, float sps, bool& clamped) {
  return clampVelocityLimit(sps, getAxisMaxVelocity(idx), clamped);
}

static float clampToMaxVelocityCeiling(float v, bool& clamped) {
  clamped = false;
  if (v > kMaxVelocityCeiling) {
    v = kMaxVelocityCeiling;
    clamped = true;
  }
  return v;
}

static void printHelp() {
  consolePrintln("Commands:");
  consolePrintln("  help/?                 -> show this help");
  consolePrintln("  setname <uart> <name>  -> set link label (by port) and push to device");
  consolePrintln("  en <axis> / dis <axis> -> enable or disable driver");
  consolePrintln("  driverstatus <axis>    -> request TMC status");
  consolePrintln("  driversettings <axis> [enable|disable] -> request settings or toggle driver");
  consolePrintln("  cur <axis> <0-100>     -> set run current %");
  consolePrintln("  ms <axis> <steps>      -> set microsteps");
  consolePrintln("  move <axis> <steps> <velocity> [accel] -> open-loop relative move (steps=full steps)");
  consolePrintln("    steps: integer, '?' (until limit toggles), or '-?'");
  consolePrintln("    Virtual axes: move x drives X1(+)/X2(-); move p drives X1(+)/X2(+)");
  consolePrintln("  moveto <axis> <deg>    -> drive to absolute angle");
  consolePrintln("  pos                    -> print current positions (x/z/p/r)");
  consolePrintln("  moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>] -> coordinated absolute move");
  consolePrintln("  home z                 -> request homing sequence (Z limit pressed)");
  consolePrintln("  coordstatus            -> show coordinated move state");
  consolePrintln("  stop [axis]            -> stop all or specific axis");
  consolePrintln("  standstillMode <axis> <normal|freewheeling|braking|strong_braking>");
  consolePrintln("  maxvelocity [axis] [sps]");
  consolePrintln("  maxaccel [axis] [sps^2]");
  consolePrintln("  showconfig             -> print motion caps");
  consolePrintln("  led <axis> <led0>.. <led7> [T=<ms>] [B=<0-255>] -> set axis LED frame");
  consolePrintln("  measure <axis> <secs>  -> sample VL6180X range on axis R");
  consolePrintln("  hi <axis>              -> display 'Hello' on OLED");
  consolePrintln("  reboot <axis>          -> reboot axis controller");
  consolePrintln("  tmcsettings <axis>     -> print TMC settings");
  consolePrintln("  tmcstatus <axis>       -> print TMC status");
}

static void stopVelocityAndMaybeDisable(size_t idx, bool disable) {
  sendVelocity(idx, 0.0f);
  if (disable) sendCmdTo(idx, (uint8_t)CommandId::Disable, 0.0f, 0.0f);
}

static void resetVelocityEstimate(Link& link) {
  link.velocity.reset();
}

static void loadPersistAll() {
  if (persistLoadV4()) {
    applyPersistToRuntime();
    return;
  }
  PersistV3 v3{};
  if (persistLoadV3(v3)) {
    g_persist = makePersistDefaultsV4();
    g_persist.cfg = v3.cfg;
    for (size_t i = 0; i < kMaxPersistLinks; ++i) {
      g_persist.axis_maxvel[i] = v3.axis_maxvel[i];
      g_persist.axis_maxvel_valid[i] = v3.axis_maxvel_valid[i];
    }
    g_persist_ok = true;
    applyPersistToRuntime();
    persistSaveV4();
    return;
  }
  // No valid persisted config yet; seed defaults and commit them.
  g_persist = makePersistDefaultsV4();
  g_persist_ok = true;
  applyPersistToRuntime();
  persistSaveV4();
}

static const char* motionModeTag(MotionMode mode) {
  switch (mode) {
    case MotionMode::MoveTo:        return "moveto";
    default:                        return "idle";
  }
}

static void completeMoveTo(size_t idx) {
  auto& link = links[idx];
  MotionState motion = link.motion;
  stopVelocityAndMaybeDisable(idx, false);
  float dur_s = (millis() - motion.start_ms) / 1000.0f;
  consolePrintfAxis(link.name, "-> plan complete (%s, %.3fs)\n", motionModeTag(motion.mode), dur_s);
  link.motion = MotionState{};
}

static void cancelMotion(size_t idx, const char* reason) {
  auto& link = links[idx];
  if (link.motion.mode != MotionMode::Idle) {
    MotionMode mode = link.motion.mode;
    stopVelocityAndMaybeDisable(idx, false);
    consolePrintfAxis(link.name, "-> cancel (%s, %s)\n", motionModeTag(mode), reason ? reason : "user");
    link.motion = MotionState{};
  } else if (reason) {
    consolePrintfAxis(link.name, "-> %s\n", reason);
  }
}

static void stopAll() {
  stopJogUntilLimit(nullptr);
  g_home.active = false;
  g_home.action_started = false;
  g_home.request_pending = false;
  coordOnStop("STOPPED");
  for (size_t i = 0; i < kNumLinks; ++i) {
    sendStopAlert(i);
    stopVelocityAndMaybeDisable(i, false);
    links[i].motion = MotionState{};
    markMoveStepsDone(i);
  }
  consolePrintfAxis("ALL", "-> stop all links\n");
}

static void stopOne(size_t idx) {
  if (idx >= kNumLinks) return;
  if (g_coord.active || g_coord_queue_count > 0) {
    stopAll();
    return;
  }
  stopJogUntilLimit(nullptr);
  g_home.active = false;
  g_home.action_started = false;
  g_home.request_pending = false;
  coordOnStop("STOPPED");
  sendStopAlert(idx);
  stopVelocityAndMaybeDisable(idx, false);
  links[idx].motion = MotionState{};
  markMoveStepsDone(idx);
  consolePrintfAxis(links[idx].name, "-> stop\n");
}

static void updateLinkVelocity(Link& link, const StatusFrame& st) {
  link.velocity.update(st);
}

static bool ensureStatusAvailable(size_t idx) {
  auto& link = links[idx];
  if (link.last_ms == 0) {
    consolePrintfAxis(link.name, "! has no status yet; wait for telemetry\n");
    return false;
  }
  return true;
}

static const char* jogAxisTag(const JogUntilLimitState& jog) {
  switch (jog.kind) {
    case JogAxisKind::VirtualX: return "x";
    case JogAxisKind::VirtualP: return "p";
    default:                   return links[jog.axis_a].name;
  }
}

static void stopJogUntilLimit(const char* reason) {
  if (!g_jog_until_limit.active) return;
  sendVelocity(g_jog_until_limit.axis_a, 0.0f);
  if (g_jog_until_limit.dual) {
    sendVelocity(g_jog_until_limit.axis_b, 0.0f);
  }
  if (reason) {
    consolePrintfAxis(jogAxisTag(g_jog_until_limit), "-> jog stop (%s)\n", reason);
  }
  g_jog_until_limit = JogUntilLimitState{};
}

static void serviceJogUntilLimit(uint32_t now_ms) {
  if (!g_jog_until_limit.active) return;

  if ((uint32_t)(now_ms - g_jog_until_limit.start_ms) > g_jog_until_limit.timeout_ms) {
    stopJogUntilLimit("timeout");
    return;
  }

  if (g_jog_until_limit.monitor_idx >= kNumLinks) {
    stopJogUntilLimit("invalid axis");
    return;
  }

  if (!isLinkConnected(links[g_jog_until_limit.monitor_idx], now_ms) ||
      !isLinkConnected(links[g_jog_until_limit.axis_a], now_ms) ||
      (g_jog_until_limit.dual && !isLinkConnected(links[g_jog_until_limit.axis_b], now_ms))) {
    stopJogUntilLimit("disconnect");
    return;
  }

  const uint8_t lim = links[g_jog_until_limit.monitor_idx].last.limit;
  if (lim != g_jog_until_limit.start_limit) {
    stopJogUntilLimit("limit");
    return;
  }
}

static bool startJogUntilLimit(JogAxisKind kind, size_t physical_idx, int8_t sign, float vel_sps) {
  if (sign != 1 && sign != -1) return false;
  if (!isfinite(vel_sps) || vel_sps <= 0.0f) return false;

  stopJogUntilLimit(nullptr);

  JogUntilLimitState jog{};
  jog.active = true;
  jog.kind = kind;
  jog.sign = sign;
  jog.vel_sps = vel_sps;
  jog.start_ms = millis();
  jog.timeout_ms = 60000;

  if (kind == JogAxisKind::Physical) {
    if (physical_idx >= kNumLinks) return false;
    jog.axis_a = physical_idx;
    jog.monitor_idx = physical_idx;
    jog.dual = false;
  } else {
    if (kNumLinks <= kIdxX2) return false;
    jog.axis_a = kIdxX1;
    jog.axis_b = kIdxX2;
    jog.dual = true;
    jog.monitor_idx = (kind == JogAxisKind::VirtualX) ? kIdxX1 : kIdxX2;
  }

  if (!ensureStatusAvailable(jog.monitor_idx) ||
      !ensureStatusAvailable(jog.axis_a) ||
      (jog.dual && !ensureStatusAvailable(jog.axis_b))) {
    return false;
  }
  jog.start_limit = links[jog.monitor_idx].last.limit ? 1 : 0;
  if (jog.start_limit && sign > 0) {
    consolePrintfAxis(jogAxisTag(jog), "! switch already triggered; use -? to release\n");
    return false;
  }
  if (!jog.start_limit && sign < 0) {
    consolePrintfAxis(jogAxisTag(jog), "! switch already released; use ? to approach\n");
    return false;
  }

  if (axisBusy(jog.axis_a)) cancelMotion(jog.axis_a, "jog");
  if (jog.dual && axisBusy(jog.axis_b)) cancelMotion(jog.axis_b, "jog");

  sendCmdTo(jog.axis_a, (uint8_t)CommandId::Enable);
  if (jog.dual) sendCmdTo(jog.axis_b, (uint8_t)CommandId::Enable);

  float vel_a = (float)sign * vel_sps;
  float vel_b = 0.0f;
  if (kind == JogAxisKind::VirtualX) {
    vel_b = (float)(-sign) * vel_sps;
  } else if (kind == JogAxisKind::VirtualP) {
    vel_b = (float)sign * vel_sps;
  }

  sendVelocity(jog.axis_a, vel_a);
  if (jog.dual) sendVelocity(jog.axis_b, vel_b);

  g_jog_until_limit = jog;
  consolePrintfAxis(jogAxisTag(jog), "-> move %s? @ %.1f sps (until switch toggles)\n", (sign < 0) ? "-" : "", vel_sps);
  return true;
}

static bool haveVirtualPair() {
  return kNumLinks > kIdxX2;
}

static float virtualXSteps(int32_t x1_steps, int32_t x2_steps) {
  return 0.5f * (float)(x1_steps - x2_steps) + g_virtual_axes.x_offset_steps;
}

static float virtualPSteps(int32_t x1_steps, int32_t x2_steps) {
  return 0.5f * (float)(x1_steps + x2_steps) + g_virtual_axes.p_offset_steps;
}

static bool virtualXYPSteps(float& out_x, float& out_p) {
  if (!haveVirtualPair()) return false;
  if (!ensureStatusAvailable(kIdxX1) || !ensureStatusAvailable(kIdxX2)) return false;
  out_x = virtualXSteps(links[kIdxX1].last.pos_steps, links[kIdxX2].last.pos_steps);
  out_p = virtualPSteps(links[kIdxX1].last.pos_steps, links[kIdxX2].last.pos_steps);
  return true;
}

static bool virtualXYPStepsQuiet(float& out_x, float& out_p) {
  if (!haveVirtualPair()) return false;
  if (links[kIdxX1].last_ms == 0 || links[kIdxX2].last_ms == 0) return false;
  out_x = virtualXSteps(links[kIdxX1].last.pos_steps, links[kIdxX2].last.pos_steps);
  out_p = virtualPSteps(links[kIdxX1].last.pos_steps, links[kIdxX2].last.pos_steps);
  return true;
}

static void markMoveStepsStarted(size_t idx) {
  if (idx >= kNumLinks) return;
  g_axis_move_active[idx] = true;
  g_axis_move_start_ms[idx] = millis();
}

static void markMoveStepsDone(size_t idx) {
  if (idx >= kNumLinks) return;
  g_axis_move_active[idx] = false;
  g_axis_move_start_ms[idx] = 0;
}

static void coordQueueClear() {
  g_coord_queue_head = 0;
  g_coord_queue_count = 0;
}

static bool coordQueuePush(const CoordRequest& req) {
  if (g_coord_queue_count >= kCoordQueueMax) return false;
  size_t tail = (g_coord_queue_head + g_coord_queue_count) % kCoordQueueMax;
  g_coord_queue[tail] = req;
  ++g_coord_queue_count;
  return true;
}

static bool coordQueuePop(CoordRequest& out) {
  if (g_coord_queue_count == 0) return false;
  out = g_coord_queue[g_coord_queue_head];
  g_coord_queue_head = (g_coord_queue_head + 1) % kCoordQueueMax;
  --g_coord_queue_count;
  return true;
}

static bool parseCoordAxisToken(const char* token, CoordAxis& axis) {
  if (!token) return false;
  if (equalsIgnoreCase(token, "x")) { axis = CoordAxis::X; return true; }
  if (equalsIgnoreCase(token, "z")) { axis = CoordAxis::Z; return true; }
  if (equalsIgnoreCase(token, "p")) { axis = CoordAxis::P; return true; }
  if (equalsIgnoreCase(token, "r")) { axis = CoordAxis::R; return true; }
  return false;
}

static void coordSetLastError(const char* code) {
  if (!code) code = "";
  strncpy(g_coord.last_error, code, sizeof(g_coord.last_error));
  g_coord.last_error[sizeof(g_coord.last_error) - 1] = '\0';
}

static void coordPrintError(const char* code, const char* detail) {
  coordSetLastError(code);
  if (detail && detail[0]) {
    consolePrintfAxis("COORD", "! err=%s %s\n", code, detail);
  } else {
    consolePrintfAxis("COORD", "! err=%s\n", code);
  }
}

static bool coordRequestHasAxis(const CoordRequest& req) {
  for (size_t i = 0; i < (size_t)CoordAxis::Count; ++i) {
    if (req.has[i]) return true;
  }
  return false;
}

static float coordMinTimeForDistance(float dist, float vmax, float accel) {
  if (!isfinite(dist) || dist <= 0.0f) return 0.0f;
  if (!isfinite(vmax) || vmax <= 0.0f) return 0.0f;
  if (!isfinite(accel) || accel <= 0.0f) return 0.0f;
  const float d_ramp = (vmax * vmax) / accel;
  if (dist <= d_ramp) {
    return 2.0f * sqrtf(dist / accel);
  }
  return (dist / vmax) + (vmax / accel);
}

static float coordPeakVelocityForTime(float dist, float accel, float total_s) {
  if (!isfinite(dist) || dist <= 0.0f) return 0.0f;
  if (!isfinite(accel) || accel <= 0.0f) return 0.0f;
  if (!isfinite(total_s) || total_s <= 0.0f) return 0.0f;
  const float at = accel * total_s;
  float disc = (at * at) - (4.0f * accel * dist);
  if (disc < 0.0f) disc = 0.0f;
  const float root = sqrtf(disc);
  return 0.5f * (at - root);
}

static bool coordOtherMotionActive() {
  if (g_home.active || g_jog_until_limit.active) return true;
  for (size_t i = 0; i < kNumLinks; ++i) {
    if (axisBusy(i)) return true;
    if (g_axis_move_active[i]) return true;
  }
  return false;
}

static void coordClearActiveState() {
  g_coord.active = false;
  g_coord.request = CoordRequest{};
  for (size_t i = 0; i < kNumLinks; ++i) {
    g_coord.axis_used[i] = false;
    g_coord.axis_done[i] = false;
    g_coord.axis_delta[i] = 0;
    g_coord.axis_vel[i] = 0.0f;
    g_coord.axis_accel[i] = 0.0f;
  }
  g_coord.start_ms = 0;
  g_coord.expected_ms = 0;
}

static void coordOnStop(const char* code) {
  if ((g_coord.active || g_coord_queue_count > 0) && code) {
    coordSetLastError(code);
  }
  coordQueueClear();
  if (g_coord.active) {
    for (size_t i = 0; i < kNumLinks; ++i) {
      if (g_coord.axis_used[i]) {
        markMoveStepsDone(i);
      }
    }
  }
  coordClearActiveState();
}

static void coordAbortActive(const char* code, const char* detail) {
  coordPrintError(code, detail);
  coordQueueClear();
  for (size_t i = 0; i < kNumLinks; ++i) {
    if (!g_coord.axis_used[i]) continue;
    if (axisBusy(i)) cancelMotion(i, "coord abort");
    sendStopAlert(i);
    sendVelocity(i, 0.0f);
    markMoveStepsDone(i);
  }
  coordClearActiveState();
}

static void coordHandleMoveDoneEvent(size_t idx) {
  markMoveStepsDone(idx);
  if (g_coord.active && idx < kNumLinks && g_coord.axis_used[idx]) {
    g_coord.axis_done[idx] = true;
  }
}

static void coordHandleMoveRejectEvent(size_t idx) {
  markMoveStepsDone(idx);
  if (g_coord.active && idx < kNumLinks && g_coord.axis_used[idx]) {
    coordAbortActive("MOVE_REJECT", links[idx].name);
  }
}

static bool coordStart(const CoordRequest& req, uint32_t now_ms) {
  const bool need_x = req.has[(size_t)CoordAxis::X];
  const bool need_z = req.has[(size_t)CoordAxis::Z];
  const bool need_p = req.has[(size_t)CoordAxis::P];
  const bool need_r = req.has[(size_t)CoordAxis::R];

  if (!coordRequestHasAxis(req)) {
    coordPrintError("NO_AXES", nullptr);
    return false;
  }

  if (need_x || need_z || need_p) {
    if (!g_soft_limits_enabled) {
      coordPrintError("NOT_HOMED", nullptr);
      return false;
    }
  }

  if (g_home.active) {
    coordPrintError("HOME_ACTIVE", nullptr);
    return false;
  }
  if (g_jog_until_limit.active) {
    coordPrintError("JOG_ACTIVE", nullptr);
    return false;
  }

  if (need_r) {
    if (links[kIdxR].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=R");
      return false;
    }
    if (!isLinkConnected(links[kIdxR], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=R");
      return false;
    }
  }

  if (need_z) {
    if (links[kIdxZ].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=Z");
      return false;
    }
    if (!isLinkConnected(links[kIdxZ], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=Z");
      return false;
    }
  }

  if (need_x || need_p) {
    if (!haveVirtualPair()) {
      coordPrintError("LINK_DOWN", "axis=XP");
      return false;
    }
    if (links[kIdxX1].last_ms == 0 || links[kIdxX2].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=XP");
      return false;
    }
    if (!isLinkConnected(links[kIdxX1], now_ms) || !isLinkConnected(links[kIdxX2], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=XP");
      return false;
    }
  }

  float cur_x = 0.0f;
  float cur_p = 0.0f;
  if (need_x || need_p) {
    if (!virtualXYPStepsQuiet(cur_x, cur_p)) {
      coordPrintError("NO_TELEM", "axis=XP");
      return false;
    }
  }

  const int32_t cur_z = links[kIdxZ].last.pos_steps;
  const int32_t cur_r = links[kIdxR].last.pos_steps;

  const float tgt_x = need_x ? (float)req.target[(size_t)CoordAxis::X] : cur_x;
  const float tgt_p = need_p ? (float)req.target[(size_t)CoordAxis::P] : cur_p;
  const int32_t tgt_z = need_z ? req.target[(size_t)CoordAxis::Z] : cur_z;
  const int32_t tgt_r = need_r ? req.target[(size_t)CoordAxis::R] : cur_r;

  if (need_x && (tgt_x < (float)kSoftLimitXMin || tgt_x > (float)kSoftLimitXMax)) {
    coordPrintError("OUT_OF_RANGE", "axis=X");
    return false;
  }
  if (need_p && (tgt_p < (float)kSoftLimitPMin || tgt_p > (float)kSoftLimitPMax)) {
    coordPrintError("OUT_OF_RANGE", "axis=P");
    return false;
  }
  if (need_z && (tgt_z < kSoftLimitZMin || tgt_z > kSoftLimitZMax)) {
    coordPrintError("OUT_OF_RANGE", "axis=Z");
    return false;
  }

  const float delta_x = tgt_x - cur_x;
  const float delta_p = tgt_p - cur_p;
  const int32_t delta_z = tgt_z - cur_z;
  const int32_t delta_r = tgt_r - cur_r;

  int32_t delta_x1 = 0;
  int32_t delta_x2 = 0;
  if (need_x || need_p) {
    delta_x1 = roundStepsSaturated(delta_x + delta_p);
    delta_x2 = roundStepsSaturated(delta_p - delta_x);
  }

  if (need_z && links[kIdxZ].last.limit != 0 && delta_z > 0) {
    coordPrintError("LIMIT_ACTIVE", "axis=Z");
    return false;
  }
  if ((need_x || need_p) && links[kIdxX1].last.limit != 0 && delta_x1 > 0) {
    coordPrintError("LIMIT_ACTIVE", "axis=X1");
    return false;
  }
  if ((need_x || need_p) && links[kIdxX2].last.limit != 0 && delta_x2 > 0) {
    coordPrintError("LIMIT_ACTIVE", "axis=X2");
    return false;
  }

  float virtual_vcap = 0.0f;
  float virtual_acap = 0.0f;
  bool have_virtual_cap = false;
  if (need_x && need_p) {
    virtual_vcap = min(getVirtualAxisMaxVelocity(VirtualAxis::X), getVirtualAxisMaxVelocity(VirtualAxis::P));
    virtual_acap = min(getVirtualAxisMaxAccel(VirtualAxis::X), getVirtualAxisMaxAccel(VirtualAxis::P));
    have_virtual_cap = true;
  } else if (need_x) {
    virtual_vcap = getVirtualAxisMaxVelocity(VirtualAxis::X);
    virtual_acap = getVirtualAxisMaxAccel(VirtualAxis::X);
    have_virtual_cap = true;
  } else if (need_p) {
    virtual_vcap = getVirtualAxisMaxVelocity(VirtualAxis::P);
    virtual_acap = getVirtualAxisMaxAccel(VirtualAxis::P);
    have_virtual_cap = true;
  }

  float vmax[kNumLinks] = {};
  float amax[kNumLinks] = {};
  float dist[kNumLinks] = {};
  bool used[kNumLinks] = {};

  if (delta_r != 0) {
    used[kIdxR] = true;
    dist[kIdxR] = fabsf((float)delta_r);
    vmax[kIdxR] = getAxisMaxVelocity(kIdxR);
    amax[kIdxR] = getAxisMaxAccel(kIdxR);
  }
  if (delta_z != 0) {
    used[kIdxZ] = true;
    dist[kIdxZ] = fabsf((float)delta_z);
    vmax[kIdxZ] = getAxisMaxVelocity(kIdxZ);
    amax[kIdxZ] = getAxisMaxAccel(kIdxZ);
  }
  if (delta_x1 != 0) {
    used[kIdxX1] = true;
    dist[kIdxX1] = fabsf((float)delta_x1);
    vmax[kIdxX1] = getAxisMaxVelocity(kIdxX1);
    amax[kIdxX1] = getAxisMaxAccel(kIdxX1);
    if (have_virtual_cap) {
      vmax[kIdxX1] = min(vmax[kIdxX1], virtual_vcap);
      amax[kIdxX1] = min(amax[kIdxX1], virtual_acap);
    }
  }
  if (delta_x2 != 0) {
    used[kIdxX2] = true;
    dist[kIdxX2] = fabsf((float)delta_x2);
    vmax[kIdxX2] = getAxisMaxVelocity(kIdxX2);
    amax[kIdxX2] = getAxisMaxAccel(kIdxX2);
    if (have_virtual_cap) {
      vmax[kIdxX2] = min(vmax[kIdxX2], virtual_vcap);
      amax[kIdxX2] = min(amax[kIdxX2], virtual_acap);
    }
  }

  float total_s = 0.0f;
  for (size_t i = 0; i < kNumLinks; ++i) {
    if (!used[i]) continue;
    const float t = coordMinTimeForDistance(dist[i], vmax[i], amax[i]);
    if (!(t > 0.0f)) {
      coordPrintError("CONFIG_INVALID", links[i].name);
      return false;
    }
    if (t > total_s) total_s = t;
  }

  if (!(total_s > 0.0f)) {
    consolePrintfAxis("COORD", "-> already at target\n");
    coordSetLastError("");
    return true;
  }

  coordClearActiveState();
  g_coord.active = true;
  g_coord.request = req;
  g_coord.start_ms = now_ms;
  g_coord.expected_ms = (uint32_t)lroundf(total_s * 1000.0f);

  for (size_t i = 0; i < kNumLinks; ++i) {
    g_coord.axis_used[i] = used[i];
    g_coord.axis_done[i] = !used[i];
  }

  g_coord.axis_delta[kIdxR] = delta_r;
  g_coord.axis_delta[kIdxZ] = delta_z;
  g_coord.axis_delta[kIdxX1] = delta_x1;
  g_coord.axis_delta[kIdxX2] = delta_x2;

  for (size_t i = 0; i < kNumLinks; ++i) {
    if (!used[i]) continue;
    float v_needed = coordPeakVelocityForTime(dist[i], amax[i], total_s);
    if (!isfinite(v_needed) || v_needed <= 0.0f) {
      coordAbortActive("CONFIG_INVALID", links[i].name);
      return false;
    }
    if (v_needed > vmax[i]) v_needed = vmax[i];
    g_coord.axis_vel[i] = v_needed;
    g_coord.axis_accel[i] = amax[i];
  }

  for (size_t i = 0; i < kNumLinks; ++i) {
    if (!used[i]) continue;
    if (axisBusy(i)) cancelMotion(i, "coord start");
    sendCmdTo(i, (uint8_t)CommandId::Enable);
    sendCmdTo(i, (uint8_t)CommandId::MoveSteps, (float)g_coord.axis_delta[i], g_coord.axis_vel[i], g_coord.axis_accel[i]);
    markMoveStepsStarted(i);
    g_coord.axis_done[i] = (g_coord.axis_delta[i] == 0);
  }

  consolePrintfAxis("COORD", "-> start t=%.3fs", total_s);
  if (need_x) consolePrintf(" x=%ld", (long)req.target[(size_t)CoordAxis::X]);
  if (need_z) consolePrintf(" z=%ld", (long)req.target[(size_t)CoordAxis::Z]);
  if (need_p) consolePrintf(" p=%ld", (long)req.target[(size_t)CoordAxis::P]);
  if (need_r) consolePrintf(" r=%ld", (long)req.target[(size_t)CoordAxis::R]);
  consolePrintln();

  coordSetLastError("");
  return true;
}

static void serviceCoordMoves(uint32_t now_ms) {
  if (g_coord.active) {
    for (size_t i = 0; i < kNumLinks; ++i) {
      if (!g_coord.axis_used[i]) continue;
      if (!isLinkConnected(links[i], now_ms)) {
        coordAbortActive("DISCONNECT", links[i].name);
        return;
      }
    }

    const uint32_t slack = max(kCoordTimeoutSlackMs,
                               (uint32_t)lroundf((float)g_coord.expected_ms * (kCoordTimeoutFactor - 1.0f)));
    if (g_coord.expected_ms > 0 &&
        (uint32_t)(now_ms - g_coord.start_ms) > (g_coord.expected_ms + slack)) {
      coordAbortActive("TIMEOUT", nullptr);
      return;
    }

    bool all_done = true;
    for (size_t i = 0; i < kNumLinks; ++i) {
      if (g_coord.axis_used[i] && !g_coord.axis_done[i]) {
        all_done = false;
        break;
      }
    }
    if (all_done) {
      float dur_s = (now_ms - g_coord.start_ms) / 1000.0f;
      consolePrintfAxis("COORD", "-> done (%.3fs)\n", dur_s);
      coordClearActiveState();
    }
  }

  if (!g_coord.active && g_coord_queue_count > 0) {
    if (coordOtherMotionActive()) return;
    CoordRequest next{};
    if (!coordQueuePop(next)) return;
    if (!coordStart(next, now_ms)) {
      coordQueueClear();
      return;
    }
  }
}

static void homeSetPhase(HomeSeqPhase phase) {
  g_home.phase = phase;
  g_home.action_started = false;
  g_home.phase_start_ms = millis();
  g_home.fixed_stable_since_ms = 0;
}

static void homeAbort(const char* why) {
  stopAll();
  g_home = HomeSeqState{};
  g_soft_limits_enabled = false;
  if (why) {
    consolePrintfAxis("HOME", "! abort: %s\n", why);
  } else {
    consolePrintfAxis("HOME", "! abort\n");
  }
}

static bool homeTargetsReached(uint32_t now_ms, int32_t tol_steps = 2, uint32_t stable_ms = 75) {
  if (!g_home.action_started) return false;
  if (g_home.fixed_a >= kNumLinks) return false;
  if (!isLinkConnected(links[g_home.fixed_a], now_ms)) return false;
  if (g_home.fixed_dual && (g_home.fixed_b >= kNumLinks || !isLinkConnected(links[g_home.fixed_b], now_ms))) {
    return false;
  }

  const int32_t cur_a = links[g_home.fixed_a].last.pos_steps;
  const bool a_ok = labs((long)(cur_a - g_home.fixed_target_a)) <= (long)tol_steps;
  bool b_ok = true;
  if (g_home.fixed_dual) {
    const int32_t cur_b = links[g_home.fixed_b].last.pos_steps;
    b_ok = labs((long)(cur_b - g_home.fixed_target_b)) <= (long)tol_steps;
  }

  const bool ok = a_ok && b_ok;
  if (!ok) {
    g_home.fixed_stable_since_ms = 0;
    return false;
  }
  if (g_home.fixed_stable_since_ms == 0) {
    g_home.fixed_stable_since_ms = now_ms;
    return false;
  }
  return (uint32_t)(now_ms - g_home.fixed_stable_since_ms) >= stable_ms;
}

static bool homeStartFixedMoveSingle(size_t idx, int32_t steps, float vel_sps, float accel_sps2) {
  if (!ensureStatusAvailable(idx)) return false;
  g_home.fixed_dual = false;
  g_home.fixed_a = idx;
  g_home.fixed_b = 0;
  g_home.fixed_target_a = links[idx].last.pos_steps + steps;
  g_home.fixed_target_b = 0;
  g_home.fixed_stable_since_ms = 0;
  g_home.action_started = true;

  sendCmdTo(idx, (uint8_t)CommandId::Enable);
  sendCmdTo(idx, (uint8_t)CommandId::MoveSteps, (float)steps, vel_sps, accel_sps2);
  return true;
}

static bool homeStartFixedMoveVirtualP(int32_t p_steps, float vel_sps, float accel_sps2) {
  if (!haveVirtualPair() || !ensureStatusAvailable(kIdxX1) || !ensureStatusAvailable(kIdxX2)) return false;
  g_home.fixed_dual = true;
  g_home.fixed_a = kIdxX1;
  g_home.fixed_b = kIdxX2;
  g_home.fixed_target_a = links[kIdxX1].last.pos_steps + p_steps;
  g_home.fixed_target_b = links[kIdxX2].last.pos_steps + p_steps;
  g_home.fixed_stable_since_ms = 0;
  g_home.action_started = true;

  sendCmdTo(kIdxX1, (uint8_t)CommandId::Enable);
  sendCmdTo(kIdxX2, (uint8_t)CommandId::Enable);
  sendCmdTo(kIdxX1, (uint8_t)CommandId::MoveSteps, (float)p_steps, vel_sps, accel_sps2);
  sendCmdTo(kIdxX2, (uint8_t)CommandId::MoveSteps, (float)p_steps, vel_sps, accel_sps2);
  return true;
}

static bool homeStartFixedMoveVirtualX(int32_t x_steps, float vel_sps, float accel_sps2) {
  if (!haveVirtualPair() || !ensureStatusAvailable(kIdxX1) || !ensureStatusAvailable(kIdxX2)) return false;
  const int32_t x1_steps = x_steps;
  const int32_t x2_steps = (int32_t)-x_steps;

  g_home.fixed_dual = true;
  g_home.fixed_a = kIdxX1;
  g_home.fixed_b = kIdxX2;
  g_home.fixed_target_a = links[kIdxX1].last.pos_steps + x1_steps;
  g_home.fixed_target_b = links[kIdxX2].last.pos_steps + x2_steps;
  g_home.fixed_stable_since_ms = 0;
  g_home.action_started = true;

  sendCmdTo(kIdxX1, (uint8_t)CommandId::Enable);
  sendCmdTo(kIdxX2, (uint8_t)CommandId::Enable);
  sendCmdTo(kIdxX1, (uint8_t)CommandId::MoveSteps, (float)x1_steps, vel_sps, accel_sps2);
  sendCmdTo(kIdxX2, (uint8_t)CommandId::MoveSteps, (float)x2_steps, vel_sps, accel_sps2);
  return true;
}

static void homeStart(uint32_t now_ms) {
  if (!haveVirtualPair()) {
    consolePrintfAxis("HOME", "! missing X1/X2 links\n");
    return;
  }

  if (!ensureStatusAvailable(kIdxZ) || !ensureStatusAvailable(kIdxX1) || !ensureStatusAvailable(kIdxX2)) {
    consolePrintfAxis("HOME", "! wait for telemetry\n");
    return;
  }

  if (!isLinkConnected(links[kIdxZ], now_ms) ||
      !isLinkConnected(links[kIdxX1], now_ms) ||
      !isLinkConnected(links[kIdxX2], now_ms)) {
    consolePrintfAxis("HOME", "! link disconnected\n");
    return;
  }

  if (links[kIdxZ].last.limit == 0) {
    consolePrintfAxis("HOME", "! precondition failed: Z switch not triggered\n");
    return;
  }

  stopAll();
  g_soft_limits_enabled = false;
  g_virtual_axes = VirtualAxesState{};

  g_home = HomeSeqState{};
  g_home.active = true;
  homeSetPhase(HomeSeqPhase::ZRelease);
  consolePrintfAxis("HOME", "-> start\n");
}

static void serviceHomeSequence(uint32_t now_ms) {
  if (!g_home.active) {
    if (g_home.request_pending) {
      g_home.request_pending = false;
      homeStart(now_ms);
    }
    return;
  }

  static constexpr uint32_t kPhaseTimeoutMs = 60000;
  if ((uint32_t)(now_ms - g_home.phase_start_ms) > kPhaseTimeoutMs) {
    homeAbort("phase timeout");
    return;
  }

  if (!isLinkConnected(links[kIdxZ], now_ms) ||
      !isLinkConnected(links[kIdxX1], now_ms) ||
      !isLinkConnected(links[kIdxX2], now_ms)) {
    homeAbort("disconnect");
    return;
  }

  switch (g_home.phase) {
    case HomeSeqPhase::Idle:
      g_home.active = false;
      return;

    case HomeSeqPhase::ZRelease: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::Physical, kIdxZ, -1, 100.0f)) {
          homeAbort("Z release start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxZ].last.limit != 0) {
        homeAbort("Z release failed");
        return;
      }
      homeSetPhase(HomeSeqPhase::ZApproach);
      return;
    }

    case HomeSeqPhase::ZApproach: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::Physical, kIdxZ, +1, 100.0f)) {
          homeAbort("Z approach start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxZ].last.limit == 0) {
        homeAbort("Z approach failed");
        return;
      }
      homeSetPhase(HomeSeqPhase::ZToTop);
      return;
    }

    case HomeSeqPhase::ZToTop: {
      if (!g_home.action_started) {
        if (!homeStartFixedMoveSingle(kIdxZ, -11550, 500.0f, 250.0f)) {
          homeAbort("Z to top start");
          return;
        }
        consolePrintfAxis("HOME", "-> Z to top\n");
        return;
      }
      if (!homeTargetsReached(now_ms)) return;
      homeSetPhase((links[kIdxX2].last.limit != 0) ? HomeSeqPhase::PRelease : HomeSeqPhase::PApproach);
      return;
    }

    case HomeSeqPhase::PRelease: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::VirtualP, 0, -1, 40.0f)) {
          homeAbort("P release start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxX2].last.limit != 0) {
        homeAbort("P release failed");
        return;
      }
      homeSetPhase(HomeSeqPhase::PApproach);
      return;
    }

    case HomeSeqPhase::PApproach: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::VirtualP, 0, +1, 40.0f)) {
          homeAbort("P approach start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxX2].last.limit == 0) {
        homeAbort("P approach failed");
        return;
      }
      homeSetPhase((links[kIdxX1].last.limit != 0) ? HomeSeqPhase::XRelease : HomeSeqPhase::XApproach);
      return;
    }

    case HomeSeqPhase::XRelease: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::VirtualX, 0, -1, 200.0f)) {
          homeAbort("X release start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxX1].last.limit != 0) {
        homeAbort("X release failed");
        return;
      }
      homeSetPhase(HomeSeqPhase::XApproach);
      return;
    }

    case HomeSeqPhase::XApproach: {
      if (!g_home.action_started) {
        if (!startJogUntilLimit(JogAxisKind::VirtualX, 0, +1, 200.0f)) {
          homeAbort("X approach start");
          return;
        }
        g_home.action_started = true;
        return;
      }
      if (g_jog_until_limit.active) return;
      if (links[kIdxX1].last.limit == 0) {
        homeAbort("X approach failed");
        return;
      }
      homeSetPhase(HomeSeqPhase::CenterP);
      return;
    }

    case HomeSeqPhase::CenterP: {
      if (!g_home.action_started) {
        if (!homeStartFixedMoveVirtualP(-255, 1000.0f, 250.0f)) {
          homeAbort("center P start");
          return;
        }
        consolePrintfAxis("HOME", "-> move P to 0\n");
        return;
      }
      if (!homeTargetsReached(now_ms)) return;
      homeSetPhase(HomeSeqPhase::CenterX);
      return;
    }

    case HomeSeqPhase::CenterX: {
      if (!g_home.action_started) {
        if (!homeStartFixedMoveVirtualX(-1050, 1000.0f, 250.0f)) {
          homeAbort("center X start");
          return;
        }
        consolePrintfAxis("HOME", "-> move X to center\n");
        return;
      }
      if (!homeTargetsReached(now_ms)) return;
      homeSetPhase(HomeSeqPhase::CenterZ);
      return;
    }

    case HomeSeqPhase::CenterZ: {
      if (!g_home.action_started) {
        if (!homeStartFixedMoveSingle(kIdxZ, 5750, 500.0f, 250.0f)) {
          homeAbort("center Z start");
          return;
        }
        consolePrintfAxis("HOME", "-> move Z to center\n");
        return;
      }
      if (!homeTargetsReached(now_ms)) return;
      g_home.active = false;
      g_soft_limits_enabled = true;
      consolePrintfAxis("HOME", "-> done (soft limits enabled)\n");
      return;
    }
  }
}

static int8_t configuredMappingSign(size_t idx) {
  if (idx < kFixedMappingSignsCount) {
    const int8_t s = kFixedMappingSigns[idx];
    if (s == 1 || s == -1) return s;
  }
  return 0; // unknown; rely on velocity feedback instead of persisted learning
}


struct MoveToPlanner {
  static void start(size_t idx, float target_deg);
  static void service(size_t idx, const StatusFrame& st);
};

void MoveToPlanner::start(size_t idx, float target_deg) {
  if (!ensureStatusAvailable(idx)) return;
  auto& link = links[idx];
  if (axisBusy(idx)) {
    consolePrintfAxis(link.name, "! busy (mode=%s) - stop first\n", motionModeTag(link.motion.mode));
    return;
  }
  float current_deg = link.last.angle_deg;
  float delta_deg = target_deg - current_deg;
  if (fabsf(delta_deg) <= kMoveToToleranceDeg) {
    consolePrintfAxis(link.name, "-> moveto already within %.2fdeg (%.3f)\n", kMoveToToleranceDeg, current_deg);
    return;
  }
  int desired_sign = (delta_deg >= 0.0f) ? +1 : -1; // desired deg/s sign
  float cruise_sps_mag = getAxisMaxVelocity(idx);
  if (!(cruise_sps_mag > 0.0f)) {
    consolePrintfAxis(link.name, "! moveto maxvelocity must be positive\n");
    return;
  }

  // Cruise fast at the max velocity; apply mapping sign if known to avoid initial flip.
  int8_t map = configuredMappingSign(idx); // +1 normal, -1 inverted, 0 unknown
  int cmd_sign = desired_sign;
  if (map == +1) cmd_sign = desired_sign;
  else if (map == -1) cmd_sign = -desired_sign;
  float velocity_sps = (float)cmd_sign * cruise_sps_mag;
  sendCmdTo(idx, (uint8_t)CommandId::Enable, 0.0f, 0.0f);
  sendVelocity(idx, velocity_sps);

  float est_duration_s = 0.0f;
  float steps_per_deg_est = estimateStepsPerDeg(link);
  if (isfinite(steps_per_deg_est) && steps_per_deg_est > 1e-3f) {
    float dps_est = fabsf(cruise_sps_mag) / steps_per_deg_est;
    if (dps_est > 1e-3f) {
      est_duration_s = fabsf(delta_deg) / dps_est;
    }
  }

  const uint32_t now_ms = millis();
  auto& motion = link.motion;
  motion = MotionState{};
  motion.mode = MotionMode::MoveTo;
  motion.start_ms = now_ms;
  motion.commanded_velocity_sps = velocity_sps;
  motion.cruise_velocity_sps = fabsf(cruise_sps_mag);
  motion.target_deg = target_deg;
  motion.tolerance_deg = kMoveToToleranceDeg;

  motion.start_error_abs = fabsf(delta_deg);
  motion.last_correction_ms = motion.start_ms;

  float abs_total = fabsf(delta_deg);
  float window_deg = max(kFinalWindowMinDeg, kFinalWindowFraction * abs_total);
  float max_window = abs_total * 0.5f;
  if (window_deg > max_window) window_deg = max_window;
  motion.slow_window_deg = window_deg;

  if (isfinite(est_duration_s) && est_duration_s > 0.0f) {
    consolePrintfAxis(link.name, "-> moveto %.2fdeg (delta %.2fdeg) in %.3fs @ %.1f sps\n",
                      target_deg,
                      delta_deg,
                      est_duration_s,
                      velocity_sps);
  } else {
    consolePrintfAxis(link.name, "-> moveto %.2fdeg (delta %.2fdeg) @ %.1f sps\n",
                      target_deg,
                      delta_deg,
                      velocity_sps);
  }
}

static void startMoveTo(size_t idx, float target_deg) { MoveToPlanner::start(idx, target_deg); }

void MoveToPlanner::service(size_t idx, const StatusFrame& st) {
  auto& link = links[idx];
  auto& motion = link.motion;
  if (motion.mode == MotionMode::MoveTo) {
    const uint32_t now_ms = millis();
    float error_deg = motion.target_deg - st.angle_deg;
    float abs_err = fabsf(error_deg);

    if (!motion.slowed) {
      float stop_gate = motion.slow_window_deg;
      if (abs_err > stop_gate) {
        // Maintain a steady cruise velocity toward the target
        int desired = (error_deg > 0.0f) ? +1 : -1; // desired deg/s sign
        float cmd_sign = (float)desired;
        int8_t map = configuredMappingSign(idx);
        if (map != 0) {
          cmd_sign = (map > 0) ? (float)desired : (float)(-desired);
        } else if (link.velocity.valid) {
          bool moving_away = (error_deg * link.velocity.dps < 0.0f);
          if (moving_away) cmd_sign = -cmd_sign;
        }
        float target_sps = cmd_sign * motion.cruise_velocity_sps;
        bool clamped_tmp = false;
        target_sps = clampVelocityAxis(idx, target_sps, clamped_tmp);
        if (fabsf(target_sps - motion.commanded_velocity_sps) >= kMinSpsDeltaToUpdate) {
          const float prev = motion.commanded_velocity_sps;
          sendVelocity(idx, target_sps);
          if ((prev > 0.0f && target_sps < 0.0f) || (prev < 0.0f && target_sps > 0.0f)) {
            motion.last_correction_ms = now_ms;
          }
          motion.commanded_velocity_sps = target_sps;
        }
        return; // keep cruising until final window
      }

      motion.slowed = true;
      motion.last_correction_ms = now_ms;
      consolePrintfAxis(link.name, "-> slowing for target (err %.2fdeg)\n", error_deg);
    }

    // Final approach: proportional-limited slow in final window
    // Direction: move to reduce error (mapping-aware if available)
    int desired = (error_deg > 0.0f) ? +1 : (error_deg < 0.0f) ? -1 : 0;
    float cmd_sign = (desired >= 0) ? +1.0f : -1.0f;
    int8_t map = configuredMappingSign(idx);
    if (map != 0) {
      cmd_sign = (map > 0) ? (float)desired : (float)(-desired);
    } else if (link.velocity.valid) {
      bool moving_away = (error_deg * link.velocity.dps < 0.0f);
      if (moving_away) cmd_sign = -cmd_sign;
    }
    float steps_per_deg_est = estimateStepsPerDeg(link);
    steps_per_deg_est = constrain(steps_per_deg_est, kStepsPerDegMinEstimate, kStepsPerDegMaxEstimate);
    float target_dps = kFinalKpDpsPerDeg * abs_err;
    if (target_dps > kFinalMaxDps) target_dps = kFinalMaxDps;
    float sps_target = cmd_sign * (target_dps * steps_per_deg_est);
    bool clamped_tmp = false;
    sps_target = clampVelocityAxis(idx, sps_target, clamped_tmp);
    if (fabsf(sps_target - motion.commanded_velocity_sps) >= kMinSpsDeltaToUpdate) {
      const float prev = motion.commanded_velocity_sps;
      sendVelocity(idx, sps_target);
      if ((prev > 0.0f && sps_target < 0.0f) || (prev < 0.0f && sps_target > 0.0f)) {
        motion.last_correction_ms = now_ms;
      }
      motion.commanded_velocity_sps = sps_target;
    }

    bool within_tol = abs_err <= motion.tolerance_deg;
    float stop_thresh = 10.0f; // faster finish while still safe
    bool stopped_ok = (link.velocity.valid && fabsf(link.velocity.dps) <= stop_thresh);
    if (within_tol && stopped_ok) {
      completeMoveTo(idx);
    }
    return;
  }
}

static void serviceTargetMotion(size_t idx, const StatusFrame& st) { MoveToPlanner::service(idx, st); }

static bool isLinkConnected(const Link& link, uint32_t now_ms) {
  return (link.last_ms != 0) && ((int32_t)(now_ms - link.last_ms) < (int32_t)kLinkDisconnectTimeoutMs);
}

static void processStatusFrame(size_t idx, const StatusFrame& st, uint32_t now_ms) {
  auto& link = links[idx];
  bool was_limit = link.last.limit;
  if (st.limit && !was_limit) {
    if (idx == kIdxZ && !g_home.active && g_soft_limits_enabled) {
      g_soft_limits_enabled = false;
      consolePrintfAxis("HOME", "! invalidated (Z limit)\n");
    }
    if (haveVirtualPair()) {
      if (idx == kIdxX2) {
        const int32_t x2_prev = link.last.pos_steps;
        const int32_t x1_now = links[kIdxX1].last.pos_steps;
        const float x_before = virtualXSteps(x1_now, x2_prev);

        if (g_home.active && g_home.phase == HomeSeqPhase::PApproach) {
          g_virtual_axes.p_offset_steps = 255.0f - 0.5f * (float)x1_now;
          g_virtual_axes.x_offset_steps = x_before - 0.5f * (float)x1_now;
        } else {
          const float delta = 0.5f * (float)x2_prev;
          g_virtual_axes.x_offset_steps -= delta;
          g_virtual_axes.p_offset_steps += delta;
        }
      } else if (idx == kIdxX1) {
        const int32_t x1_prev = link.last.pos_steps;
        const int32_t x2_now = links[kIdxX2].last.pos_steps;
        const float p_before = virtualPSteps(x1_prev, x2_now);

        if (g_home.active && g_home.phase == HomeSeqPhase::XApproach) {
          g_virtual_axes.x_offset_steps = 2150.0f + 0.5f * (float)x2_now;
          g_virtual_axes.p_offset_steps = p_before - 0.5f * (float)x2_now;
        } else {
          const float delta = 0.5f * (float)x1_prev;
          g_virtual_axes.x_offset_steps += delta;
          g_virtual_axes.p_offset_steps += delta;
        }
      }
    }

    // Halt any active plan when limit is hit to allow new commands
    cancelMotion(idx, "limit switch");
    sendStopAlert(idx);
    if (g_coord.active && g_coord.axis_used[idx]) {
      coordAbortActive("LIMIT_HIT", links[idx].name);
    }
  }
  StatusFrame copy = st;
  updateLinkVelocity(link, st);
  link.last = copy;
  link.last_ms = now_ms;
  serviceTargetMotion(idx, link.last);
}

static void processLinkInput(size_t idx) {
  auto& link = links[idx];
  uint8_t type;
  uint8_t buf[64];
  while (link.port->available()) {
    uint16_t cap = sizeof(buf);
    if (!recvMessage(*link.port, type, buf, cap)) break;
    if (type == MSG_STATUS && cap == sizeof(StatusFrame)) {
      const StatusFrame* st = reinterpret_cast<const StatusFrame*>(buf);
      uint32_t now_ms = millis();
      processStatusFrame(idx, *st, now_ms);
    } else if (type == MSG_EVENT && cap > 0) {
      char msg[64] = {};
      size_t n = (cap < sizeof(msg) - 1) ? cap : (sizeof(msg) - 1);
      memcpy(msg, buf, n);
      msg[n] = '\0';
      if (!strncmp(msg, "-> move done", 12)) {
        coordHandleMoveDoneEvent(idx);
      } else if (!strncmp(msg, "move.reject", 11)) {
        coordHandleMoveRejectEvent(idx);
      }
      if (idx == kIdxZ) {
        static constexpr char kHomeRequestEvent[] = "home.request";
        if (cap == (sizeof(kHomeRequestEvent) - 1) &&
            memcmp(buf, kHomeRequestEvent, sizeof(kHomeRequestEvent) - 1) == 0) {
          if (!g_home.active) {
            g_home.request_pending = true;
          }
        }
      }
      consolePrintf("%-8s ", link.name);
      for (uint16_t k = 0; k < cap; ++k) consoleWrite(buf[k]);
      consolePrintln();
    }
  }
}

static void handleStopButton() {
  bool stop_pressed = (digitalRead(kStopButtonPin) == LOW);
  if (stop_pressed && !g_stop_button_prev_pressed) {
    uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - g_stop_button_last_trigger_ms) >= kStopButtonDebounceMs) {
      g_stop_button_last_trigger_ms = now_ms;
      stopAll();
    }
  }
  g_stop_button_prev_pressed = stop_pressed;
}

static void handleConnectionTransition(size_t idx, bool connected) {
  auto& link = links[idx];
  if (connected && !link.was_connected) {
    pushLinkName(idx);
    consolePrintf("%-8s reconnected\n", link.name);
    resetVelocityEstimate(link);
  } else if (!connected && link.was_connected) {
    cancelMotion(idx, "disconnect");
    resetVelocityEstimate(link);
    markMoveStepsDone(idx);
    if (g_coord.active && g_coord.axis_used[idx]) {
      coordAbortActive("DISCONNECT", link.name);
    }
    if (g_soft_limits_enabled && (idx == kIdxZ || idx == kIdxX1 || idx == kIdxX2)) {
      g_soft_limits_enabled = false;
      consolePrintfAxis("HOME", "! invalidated (link disconnect)\n");
    }
  }
  link.was_connected = connected;
}

static void printLinkStatusLine(size_t idx, uint32_t now_ms) {
  auto& link = links[idx];
  StatusFrame st = link.last;
  char ts_field[16];
  snprintf(ts_field, sizeof(ts_field), "%lu", (unsigned long)st.ts_us);

  char ang_field[16];
  snprintf(ang_field, sizeof(ang_field), "%0.2f", st.angle_deg);

  char dist_field[16];
  snprintf(dist_field, sizeof(dist_field), "%ld", (long)st.pos_steps);

  char dps_field[16];
  if (link.velocity.valid) {
    snprintf(dps_field, sizeof(dps_field), "%0.2f", link.velocity.dps);
  } else {
    strncpy(dps_field, "---", sizeof(dps_field));
    dps_field[sizeof(dps_field) - 1] = '\0';
  }

  char temp_field[16];
  if (isfinite(st.temp_c) && st.temp_c > -55.0f && st.temp_c < 125.0f) {
    snprintf(temp_field, sizeof(temp_field), "%0.1f", st.temp_c);
  } else {
    strncpy(temp_field, "---", sizeof(temp_field));
    temp_field[sizeof(temp_field) - 1] = '\0';
  }

  consolePrintf("%-8s   ts:%s  ang:%s  dps:%s  dist:%s  temp:%s  lim:%u  drv:%s",
                link.name,
                ts_field,
                ang_field,
                dps_field,
                dist_field,
                temp_field,
                (unsigned)st.limit,
                st.driver ? "ON" : "OFF");

  if (st.fault) {
    consolePrintf("  flt:%02X", (unsigned)st.fault);
  }
  if (link.motion.mode == MotionMode::MoveTo) {
    float err = link.motion.target_deg - st.angle_deg;
    consolePrintf(" tgt:%0.2f err:%+.2f", link.motion.target_deg, err);

    // Debug: show the current steps/deg calibration estimate only while doing MoveTo.
    // A trailing '?' means we're falling back to a default because the velocity estimate isn't reliable.
    bool cal_measured = false;
    if (link.velocity.valid && fabsf(link.velocity.dps) >= kDirectionCheckMinDps) {
      const float cmd = fabsf(link.motion.commanded_velocity_sps);
      const float dps = fabsf(link.velocity.dps);
      cal_measured = (cmd > 1e-3f) && (dps > 1e-3f);
    }
    const float cal = estimateStepsPerDeg(link);
    consolePrintf(" cal:%0.3f%s", cal, cal_measured ? "" : "?");
  }
  if (st.fault) {
    consolePrint(" [");
    if (st.fault & (1u << 1)) consolePrint("DRV ");
    if (st.fault & (1u << 2)) consolePrint("UVCP ");
    if (st.fault & (1u << 3)) consolePrint("OTW ");
    if (st.fault & (1u << 4)) consolePrint("OT ");
    if (st.fault & (1u << 5)) consolePrint("S2G ");
    if (st.fault & (1u << 6)) consolePrint("LS ");
    if (st.fault & (1u << 7)) consolePrint("OL ");
    consolePrint("]");
  }
  consolePrintln();
}

static int32_t roundStepsSaturated(float v) {
  if (!isfinite(v)) return 0;
  if (v > (float)INT32_MAX) v = (float)INT32_MAX;
  if (v < (float)INT32_MIN) v = (float)INT32_MIN;
  return (int32_t)lroundf(v);
}

static float mergeTempC(float a, float b) {
  auto valid = [](float t) {
    return isfinite(t) && t > -55.0f && t < 125.0f;
  };
  const bool a_ok = valid(a);
  const bool b_ok = valid(b);
  if (a_ok && b_ok) return 0.5f * (a + b);
  if (a_ok) return a;
  if (b_ok) return b;
  return NAN;
}

static void printVirtualAxisStatusLine(const char* axis_name,
                                       uint32_t ts_us,
                                       float angle_deg,
                                       bool dps_valid,
                                       float dps,
                                       int32_t pos_steps,
                                       float temp_c,
                                       uint8_t limit,
                                       uint8_t fault,
                                       uint8_t driver) {
  char ts_field[16];
  snprintf(ts_field, sizeof(ts_field), "%lu", (unsigned long)ts_us);

  char ang_field[16];
  snprintf(ang_field, sizeof(ang_field), "%0.2f", angle_deg);

  char dist_field[16];
  snprintf(dist_field, sizeof(dist_field), "%ld", (long)pos_steps);

  char dps_field[16];
  if (dps_valid && isfinite(dps)) {
    snprintf(dps_field, sizeof(dps_field), "%0.2f", dps);
  } else {
    strncpy(dps_field, "---", sizeof(dps_field));
    dps_field[sizeof(dps_field) - 1] = '\0';
  }

  char temp_field[16];
  if (isfinite(temp_c) && temp_c > -55.0f && temp_c < 125.0f) {
    snprintf(temp_field, sizeof(temp_field), "%0.1f", temp_c);
  } else {
    strncpy(temp_field, "---", sizeof(temp_field));
    temp_field[sizeof(temp_field) - 1] = '\0';
  }

  consolePrintf("%-8s   ts:%s  ang:%s  dps:%s  dist:%s  temp:%s  lim:%u  drv:%s",
                axis_name,
                ts_field,
                ang_field,
                dps_field,
                dist_field,
                temp_field,
                (unsigned)limit,
                driver ? "ON" : "OFF");

  if (fault) {
    consolePrintf("  flt:%02X", (unsigned)fault);
  }
  consolePrintln();
}

static void printLinksIfDue(uint32_t now_ms) {
  static uint32_t nextPrint = 0;
  if ((int32_t)(now_ms - nextPrint) < 0) return;
  nextPrint = now_ms + 250;
  bool connected[kNumLinks] = {};
  for (size_t i = 0; i < kNumLinks; ++i) {
    connected[i] = isLinkConnected(links[i], now_ms);
    handleConnectionTransition(i, connected[i]);
  }

  for (size_t i = 0; i < kNumLinks; ++i) {
    if (!connected[i]) continue;
    if (i == kIdxX1 || i == kIdxX2) continue;
    printLinkStatusLine(i, now_ms);
  }

  if (kNumLinks > kIdxX2 && connected[kIdxX1] && connected[kIdxX2]) {
    const StatusFrame x1 = links[kIdxX1].last;
    const StatusFrame x2 = links[kIdxX2].last;

    const float x_steps = 0.5f * (float)(x1.pos_steps - x2.pos_steps) + g_virtual_axes.x_offset_steps;
    const float p_steps = 0.5f * (float)(x1.pos_steps + x2.pos_steps) + g_virtual_axes.p_offset_steps;

    const uint32_t ts_us = (x1.ts_us > x2.ts_us) ? x1.ts_us : x2.ts_us;
    const float angle_deg = 0.5f * (x1.angle_deg + x2.angle_deg);
    const float temp_c = mergeTempC(x1.temp_c, x2.temp_c);
    const uint8_t fault = (uint8_t)(x1.fault | x2.fault);
    const uint8_t driver = (x1.driver && x2.driver) ? 1 : 0;

    printVirtualAxisStatusLine("x",
                              ts_us,
                              angle_deg,
                              false,
                              0.0f,
                              roundStepsSaturated(x_steps),
                              temp_c,
                              x1.limit,
                              fault,
                              driver);

    printVirtualAxisStatusLine("p",
                              ts_us,
                              angle_deg,
                              false,
                              0.0f,
                              roundStepsSaturated(p_steps),
                              temp_c,
                              x2.limit,
                              fault,
                              driver);
  }
}

static void sendPingsIfDue(uint32_t now_ms) {
  static uint32_t nextPing = 0;
  if ((int32_t)(now_ms - nextPing) < 0) return;
  nextPing = now_ms + kLinkPingIntervalMs;
  for (size_t i = 0; i < kNumLinks; ++i) {
    CommandFrame ping{};
    ping.axis_id = 0;
    ping.cmd = (uint8_t)CommandId::Ping;
    ping.p0 = 0.0f;
    ping.p1 = 0.0f;
    sendCommand(*links[i].port, ping);
  }
}

static constexpr const char* kCmdDelims = " \t\r\n";

static char* nextToken(char*& save) { return strtok_r(nullptr, kCmdDelims, &save); }

using ConsoleCmdHandler = void (*)(char* save);

static void cmdHelp(char*) { printHelp(); }

static void setDriverEnabled(size_t idx, bool enable) {
  if (enable) {
    sendCmdTo(idx, (uint8_t)CommandId::Enable);
    consolePrintfAxis(links[idx].name, "-> enable\n");
    return;
  }
  if (axisBusy(idx)) {
    cancelMotion(idx, "disable");
  }
  stopVelocityAndMaybeDisable(idx, true);
  links[idx].motion = MotionState{};
  markMoveStepsDone(idx);
  consolePrintfAxis(links[idx].name, "-> disable\n");
}

static void cmdEnable(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: en <axis>");
    return;
  }
  setDriverEnabled(idx, true);
}

static void cmdDisable(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: dis <axis>");
    return;
  }
  setDriverEnabled(idx, false);
}

static void cmdDriverStatus(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: driverstatus <axis>");
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::TmcStatus);
}

static void cmdDriverSettings(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  char* action = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: driversettings <axis> [enable|disable]");
    return;
  }
  if (!action) {
    sendCmdTo(idx, (uint8_t)CommandId::TmcSettings);
    return;
  }
  if (equalsIgnoreCase(action, "enable")) {
    setDriverEnabled(idx, true);
    return;
  }
  if (equalsIgnoreCase(action, "disable")) {
    setDriverEnabled(idx, false);
    return;
  }
  consolePrintln("Usage: driversettings <axis> [enable|disable]");
}

static void cmdSetName(char* save) {
  char* savep = save;
  char* t1 = nextToken(savep);
  char* t2 = nextToken(savep);
  if (!t1 || !t2) {
    consolePrintln("Usage: setname <uart> <name>");
    return;
  }
  long port_idx = strtol(t1, nullptr, 10);
  if (port_idx < 1 || (size_t)port_idx > kNumLinks) {
    consolePrintln("Out of range");
    return;
  }
  size_t link_idx = (size_t)(port_idx - 1);
  bool truncated = (strlen(t2) > kLinkNameMaxLen);
  setLinkName(link_idx, t2);
  consolePrintfAxis(links[link_idx].name, "-> setname UART%ld\n", port_idx);
  if (truncated) {
    consolePrintfAxis(links[link_idx].name, "! name truncated to 8 characters\n");
  }
}

static void cmdCurrent(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  char* t2 = nextToken(savep);
  size_t idx = 0;
  if (!axis || !t2 || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: cur <axis> <0-100>");
    return;
  }
  float pct = strtof(t2, nullptr);
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  sendCmdTo(idx, (uint8_t)CommandId::SetCurrent, pct);
  consolePrintfAxis(links[idx].name, "-> current %.1f%%\n", pct);
}

static void cmdMicrosteps(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  char* t2 = nextToken(savep);
  size_t idx = 0;
  if (!axis || !t2 || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: ms <axis> <microsteps>");
    return;
  }
  float ms = (float)strtoul(t2, nullptr, 0);
  sendCmdTo(idx, (uint8_t)CommandId::SetMicrosteps, ms);
  consolePrintfAxis(links[idx].name, "-> microsteps %u\n", (unsigned)ms);
}

static void cmdMove(char* save) {
  if (g_home.active) {
    consolePrintfAxis("HOME", "! busy - stop first\n");
    return;
  }
  if (g_coord.active || g_coord_queue_count > 0) {
    consolePrintfAxis("COORD", "! busy - stop first\n");
    return;
  }
  char* savep = save;
  char* axis = nextToken(savep);
  char* t_steps = nextToken(savep);
  char* t_vel = nextToken(savep);
  char* t_acc = nextToken(savep);

  static constexpr size_t kVirtualX1 = 2; // links[] index for physical X1 actuator
  static constexpr size_t kVirtualX2 = 3; // links[] index for physical X2 actuator

  const bool is_virtual_x = axis && equalsIgnoreCase(axis, "x");
  const bool is_virtual_p = axis && equalsIgnoreCase(axis, "p");
  const bool is_seek_pos = t_steps && (equalsIgnoreCase(t_steps, "?") || equalsIgnoreCase(t_steps, "+?"));
  const bool is_seek_neg = t_steps && equalsIgnoreCase(t_steps, "-?");
  const bool is_seek = is_seek_pos || is_seek_neg;

  size_t idx = 0;
  if (!axis || !t_steps || !t_vel || (!is_virtual_x && !is_virtual_p && !findLinkIndexByName(axis, idx))) {
    consolePrintln("Usage: move <axis> <steps> <velocity_sps> [accel_sps^2]");
    return;
  }
  const char* axis_tag = is_virtual_x ? "X" : (is_virtual_p ? "P" : links[idx].name);

  float vel_sps = fabsf(strtof(t_vel, nullptr));
  if (!isfinite(vel_sps) || vel_sps <= 0.0f) {
    consolePrintfAxis(axis_tag, "! move velocity must be >0\n");
    return;
  }

  // Move commands intentionally ignore maxvelocity/maxaccel (those are tuned for MoveTo).
  // If accel is omitted, send 0 and let the axis firmware pick a reasonable default.
  float accel_sps2 = 0.0f;
  if (t_acc) {
    accel_sps2 = fabsf(strtof(t_acc, nullptr));
    if (!isfinite(accel_sps2) || accel_sps2 <= 0.0f) {
      consolePrintfAxis(axis_tag, "! move accel must be >0\n");
      return;
    }
  }

  if (is_seek) {
    const int8_t sign = is_seek_pos ? +1 : -1;
    const bool ok = startJogUntilLimit(is_virtual_x ? JogAxisKind::VirtualX
                                                    : (is_virtual_p ? JogAxisKind::VirtualP : JogAxisKind::Physical),
                                       idx,
                                       sign,
                                       vel_sps);
    if (!ok) {
      consolePrintfAxis(axis_tag, "! move failed\n");
    }
    return;
  }

  stopJogUntilLimit(nullptr);

  long steps_long = strtol(t_steps, nullptr, 0);
  if (steps_long < (long)INT32_MIN) steps_long = (long)INT32_MIN;
  if (steps_long > (long)INT32_MAX) steps_long = (long)INT32_MAX;
  int32_t steps = (int32_t)steps_long;
  if (steps == 0) {
    consolePrintfAxis(axis_tag, "! move steps must be non-zero\n");
    return;
  }

  if (!is_virtual_x && !is_virtual_p) {
    if (!ensureStatusAvailable(idx)) return;
    if (idx == kIdxZ && links[kIdxZ].last.limit != 0 && steps > 0) {
      consolePrintfAxis(links[idx].name, "! blocked by limit switch\n");
      return;
    }
    if (g_soft_limits_enabled && idx == kIdxZ) {
      const int32_t z = links[kIdxZ].last.pos_steps;
      const int32_t next_z = z + steps;
      if (next_z < kSoftLimitZMin || next_z > kSoftLimitZMax) {
        consolePrintfAxis(links[idx].name, "! blocked by soft limit (z: [%ld, %ld])\n",
                          (long)kSoftLimitZMin,
                          (long)kSoftLimitZMax);
        return;
      }
    }
  }

  if (is_virtual_x || is_virtual_p) {
    if (kNumLinks <= kVirtualX2) {
      consolePrintfAxis("X/P", "! virtual axes require X1+X2 links\n");
      return;
    }
    if (!ensureStatusAvailable(kVirtualX1) || !ensureStatusAvailable(kVirtualX2)) {
      return;
    }

    const int32_t steps_x1 = steps;
    const int32_t steps_x2 = is_virtual_x ? (int32_t)-steps : steps;

    if (is_virtual_x && links[kVirtualX1].last.limit != 0 && steps > 0) {
      consolePrintfAxis("x", "! blocked by X1 limit switch\n");
      return;
    }
    if (is_virtual_p && links[kVirtualX2].last.limit != 0 && steps > 0) {
      consolePrintfAxis("p", "! blocked by X2 limit switch\n");
      return;
    }

    if (g_soft_limits_enabled) {
      float x = 0.0f;
      float p = 0.0f;
      if (!virtualXYPSteps(x, p)) {
        consolePrintfAxis(is_virtual_x ? "x" : "p", "! no virtual telemetry\n");
        return;
      }
      if (is_virtual_x) {
        float next = x + (float)steps;
        if (next < (float)kSoftLimitXMin || next > (float)kSoftLimitXMax) {
          consolePrintfAxis("x", "! blocked by soft limit (x: [%ld, %ld])\n",
                            (long)kSoftLimitXMin,
                            (long)kSoftLimitXMax);
          return;
        }
      } else {
        float next = p + (float)steps;
        if (next < (float)kSoftLimitPMin || next > (float)kSoftLimitPMax) {
          consolePrintfAxis("p", "! blocked by soft limit (p: [%ld, %ld])\n",
                            (long)kSoftLimitPMin,
                            (long)kSoftLimitPMax);
          return;
        }
      }
    }

    if (axisBusy(kVirtualX1)) cancelMotion(kVirtualX1, is_virtual_x ? "move x" : "move p");
    if (axisBusy(kVirtualX2)) cancelMotion(kVirtualX2, is_virtual_x ? "move x" : "move p");

    sendCmdTo(kVirtualX1, (uint8_t)CommandId::Enable);
    sendCmdTo(kVirtualX2, (uint8_t)CommandId::Enable);
    sendCmdTo(kVirtualX1, (uint8_t)CommandId::MoveSteps, (float)steps_x1, vel_sps, accel_sps2);
    sendCmdTo(kVirtualX2, (uint8_t)CommandId::MoveSteps, (float)steps_x2, vel_sps, accel_sps2);
    markMoveStepsStarted(kVirtualX1);
    markMoveStepsStarted(kVirtualX2);

    const char* tag = is_virtual_x ? "X" : "P";
    if (t_acc) {
      consolePrintfAxis(tag, "-> move X1:%ld X2:%ld @ %.1f sps accel %.1f\n",
                        (long)steps_x1,
                        (long)steps_x2,
                        vel_sps,
                        accel_sps2);
    } else {
      consolePrintfAxis(tag, "-> move X1:%ld X2:%ld @ %.1f sps\n",
                        (long)steps_x1,
                        (long)steps_x2,
                        vel_sps);
    }
    return;
  }

  if (axisBusy(idx)) cancelMotion(idx, "move");
  sendCmdTo(idx, (uint8_t)CommandId::Enable);
  sendCmdTo(idx, (uint8_t)CommandId::MoveSteps, (float)steps, vel_sps, accel_sps2);
  markMoveStepsStarted(idx);

  if (t_acc) {
    consolePrintfAxis(links[idx].name, "-> move %ld steps @ %.1f sps accel %.1f\n",
                      (long)steps,
                      vel_sps,
                      accel_sps2);
  } else {
    consolePrintfAxis(links[idx].name, "-> move %ld steps @ %.1f sps\n",
                      (long)steps,
                      vel_sps);
  }
}

static void cmdMoveTo(char* save) {
  if (g_home.active) {
    consolePrintfAxis("HOME", "! busy - stop first\n");
    return;
  }
  if (g_coord.active || g_coord_queue_count > 0) {
    consolePrintfAxis("COORD", "! busy - stop first\n");
    return;
  }
  char* savep = save;
  char* axis = nextToken(savep);
  char* t1 = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx) || !t1) {
    consolePrintln("Usage: moveto <axis> <degrees>");
    return;
  }
  float target = strtof(t1, nullptr);
  startMoveTo(idx, target);
}

static void cmdPos(char*) {
  uint32_t now_ms = millis();
  if (links[kIdxR].last_ms == 0 || !isLinkConnected(links[kIdxR], now_ms)) {
    consolePrintfAxis("POS", "! err=NO_TELEM axis=R\n");
    return;
  }
  if (links[kIdxZ].last_ms == 0 || !isLinkConnected(links[kIdxZ], now_ms)) {
    consolePrintfAxis("POS", "! err=NO_TELEM axis=Z\n");
    return;
  }
  if (!haveVirtualPair() ||
      links[kIdxX1].last_ms == 0 ||
      links[kIdxX2].last_ms == 0 ||
      !isLinkConnected(links[kIdxX1], now_ms) ||
      !isLinkConnected(links[kIdxX2], now_ms)) {
    consolePrintfAxis("POS", "! err=NO_TELEM axis=XP\n");
    return;
  }

  float x = 0.0f;
  float p = 0.0f;
  if (!virtualXYPStepsQuiet(x, p)) {
    consolePrintfAxis("POS", "! err=NO_TELEM axis=XP\n");
    return;
  }

  const int32_t x_steps = roundStepsSaturated(x);
  const int32_t z_steps = links[kIdxZ].last.pos_steps;
  const int32_t p_steps = roundStepsSaturated(p);
  const int32_t r_steps = links[kIdxR].last.pos_steps;

  consolePrintfAxis("POS", "x:%ld z:%ld p:%ld r:%ld homed:%u\n",
                    (long)x_steps,
                    (long)z_steps,
                    (long)p_steps,
                    (long)r_steps,
                    g_soft_limits_enabled ? 1u : 0u);
}

static void cmdMoveAbs(char* save) {
  CoordRequest req{};
  char* savep = save;
  char* token = nullptr;
  while ((token = nextToken(savep)) != nullptr) {
    char* key = token;
    char* value = nullptr;
    char* eq = strchr(token, '=');
    if (eq) {
      *eq = '\0';
      value = eq + 1;
    } else {
      value = nextToken(savep);
    }
    if (!value) {
      consolePrintln("Usage: moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]");
      return;
    }

    CoordAxis axis = CoordAxis::X;
    if (!parseCoordAxisToken(key, axis)) {
      consolePrintln("Usage: moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]");
      return;
    }
    size_t axis_idx = (size_t)axis;
    if (axis_idx >= (size_t)CoordAxis::Count) {
      consolePrintln("Usage: moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]");
      return;
    }
    if (req.has[axis_idx]) {
      coordPrintError("DUPLICATE_AXIS", key);
      return;
    }
    long v = strtol(value, nullptr, 0);
    if (v < (long)INT32_MIN) v = (long)INT32_MIN;
    if (v > (long)INT32_MAX) v = (long)INT32_MAX;
    req.target[axis_idx] = (int32_t)v;
    req.has[axis_idx] = true;
  }

  if (!coordRequestHasAxis(req)) {
    consolePrintln("Usage: moveabs [x <steps>] [z <steps>] [p <steps>] [r <steps>]");
    return;
  }

  if (req.has[(size_t)CoordAxis::X] || req.has[(size_t)CoordAxis::Z] || req.has[(size_t)CoordAxis::P]) {
    if (!g_soft_limits_enabled) {
      coordPrintError("NOT_HOMED", nullptr);
      return;
    }
  }

  if (g_home.active) {
    coordPrintError("HOME_ACTIVE", nullptr);
    return;
  }
  if (g_jog_until_limit.active) {
    coordPrintError("JOG_ACTIVE", nullptr);
    return;
  }

  if (req.has[(size_t)CoordAxis::X]) {
    int32_t x = req.target[(size_t)CoordAxis::X];
    if (x < kSoftLimitXMin || x > kSoftLimitXMax) {
      coordPrintError("OUT_OF_RANGE", "axis=X");
      return;
    }
  }
  if (req.has[(size_t)CoordAxis::P]) {
    int32_t p = req.target[(size_t)CoordAxis::P];
    if (p < kSoftLimitPMin || p > kSoftLimitPMax) {
      coordPrintError("OUT_OF_RANGE", "axis=P");
      return;
    }
  }
  if (req.has[(size_t)CoordAxis::Z]) {
    int32_t z = req.target[(size_t)CoordAxis::Z];
    if (z < kSoftLimitZMin || z > kSoftLimitZMax) {
      coordPrintError("OUT_OF_RANGE", "axis=Z");
      return;
    }
  }

  const uint32_t now_ms = millis();
  if (req.has[(size_t)CoordAxis::R]) {
    if (links[kIdxR].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=R");
      return;
    }
    if (!isLinkConnected(links[kIdxR], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=R");
      return;
    }
  }
  if (req.has[(size_t)CoordAxis::Z]) {
    if (links[kIdxZ].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=Z");
      return;
    }
    if (!isLinkConnected(links[kIdxZ], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=Z");
      return;
    }
  }
  if (req.has[(size_t)CoordAxis::X] || req.has[(size_t)CoordAxis::P]) {
    if (!haveVirtualPair()) {
      coordPrintError("LINK_DOWN", "axis=XP");
      return;
    }
    if (links[kIdxX1].last_ms == 0 || links[kIdxX2].last_ms == 0) {
      coordPrintError("NO_TELEM", "axis=XP");
      return;
    }
    if (!isLinkConnected(links[kIdxX1], now_ms) || !isLinkConnected(links[kIdxX2], now_ms)) {
      coordPrintError("LINK_DOWN", "axis=XP");
      return;
    }
  }

  const bool busy = g_coord.active || coordOtherMotionActive() || (g_coord_queue_count > 0);
  if (busy) {
    if (!coordQueuePush(req)) {
      coordPrintError("QUEUE_FULL", nullptr);
      return;
    }
    consolePrintfAxis("COORD", "-> queued (len=%u)\n", (unsigned)g_coord_queue_count);
    return;
  }

  coordStart(req, now_ms);
}

static void cmdHome(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  if (!axis || !equalsIgnoreCase(axis, "z")) {
    consolePrintln("Usage: home z");
    return;
  }
  if (!g_home.active) {
    g_home.request_pending = true;
  }
}

static void cmdCoordStatus(char*) {
  const char* state = g_coord.active ? "running" : (g_coord_queue_count > 0 ? "queued" : "idle");
  const char* err = (g_coord.last_error[0] != '\0') ? g_coord.last_error : "NONE";
  char axes[8] = "-";
  if (g_coord.active) {
    size_t n = 0;
    if (g_coord.request.has[(size_t)CoordAxis::X]) axes[n++] = 'X';
    if (g_coord.request.has[(size_t)CoordAxis::Z]) axes[n++] = 'Z';
    if (g_coord.request.has[(size_t)CoordAxis::P]) axes[n++] = 'P';
    if (g_coord.request.has[(size_t)CoordAxis::R]) axes[n++] = 'R';
    if (n == 0) axes[n++] = '-';
    axes[n] = '\0';
  }
  consolePrintfAxis("COORD", "state=%s homed=%u queue=%u err=%s axes=%s\n",
                    state,
                    g_soft_limits_enabled ? 1u : 0u,
                    (unsigned)g_coord_queue_count,
                    err,
                    axes);
}

static void cmdStop(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  if (!axis) {
    stopAll();
    return;
  }
  size_t idx = 0;
  if (!findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: stop [axis]");
    return;
  }
  stopOne(idx);
}

static void cmdStandstillMode(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  char* tm = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx) || !tm) {
    consolePrintln(kStandstillModeUsage);
    return;
  }
  uint8_t mode = 0;
  if (!parseStandstillModeToken(tm, mode)) {
    consolePrintln(kStandstillModeUsage);
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::SetStandstillMode, (float)mode, 0.0f);
  consolePrintfAxis(links[idx].name, "-> standstillMode %s\n", standstillModeName(mode));
}

static void cmdMaxVelocity(char* save) {
  char* savep = save;
  char* t1 = nextToken(savep);
  if (!t1) {
    consolePrintf("maxvelocity: %.1f sps\n", min(g_config.max_velocity, kMaxVelocityCeiling));
    return;
  }
  size_t axis_idx = 0;
  AxisTokenKind kind = AxisTokenKind::None;
  bool has_axis = parseAxisToken(t1, kind, axis_idx);
  char* value_token = has_axis ? nextToken(savep) : t1;
  if (!value_token) {
    if (has_axis) {
      if (kind == AxisTokenKind::Physical) {
        consolePrintfAxis(links[axis_idx].name, "maxvelocity: %.1f sps\n", getAxisMaxVelocity(axis_idx));
      } else if (kind == AxisTokenKind::VirtualX) {
        consolePrintfAxis("x", "maxvelocity: %.1f sps\n", getVirtualAxisMaxVelocity(VirtualAxis::X));
      } else if (kind == AxisTokenKind::VirtualP) {
        consolePrintfAxis("p", "maxvelocity: %.1f sps\n", getVirtualAxisMaxVelocity(VirtualAxis::P));
      }
    } else {
      consolePrintf("maxvelocity: %.1f sps\n", min(g_config.max_velocity, kMaxVelocityCeiling));
    }
    return;
  }
  float v = strtof(value_token, nullptr);
  if (!isfinite(v) || v <= 0.0f) {
    if (has_axis) {
      consolePrintfAxis((kind == AxisTokenKind::VirtualX) ? "x" :
                        (kind == AxisTokenKind::VirtualP) ? "p" : links[axis_idx].name,
                        "! maxvelocity must be >0\n");
    } else {
      consolePrintln("! maxvelocity must be >0");
    }
    return;
  }
  bool ceiling_clamped = false;
  v = clampToMaxVelocityCeiling(v, ceiling_clamped);
  if (has_axis) {
    if (kind == AxisTokenKind::Physical) {
      if (axis_idx < kMaxPersistLinks) {
        g_axis_max_velocity[axis_idx] = v;
        g_axis_max_velocity_valid[axis_idx] = true;
        saveAxisVelocityFor(axis_idx);
      }
      consolePrintfAxis(links[axis_idx].name, "-> maxvelocity %.1f sps\n", getAxisMaxVelocity(axis_idx));
      if (ceiling_clamped) {
        consolePrintfAxis(links[axis_idx].name, "! capped to global ceiling %.1f sps\n", kMaxVelocityCeiling);
      }
    } else {
      size_t v_idx = (kind == AxisTokenKind::VirtualX) ? 0u : 1u;
      g_virtual_max_velocity[v_idx] = v;
      g_virtual_max_velocity_valid[v_idx] = true;
      if (kIdxX1 < kMaxPersistLinks) {
        g_axis_max_velocity[kIdxX1] = v;
        g_axis_max_velocity_valid[kIdxX1] = true;
      }
      if (kIdxX2 < kMaxPersistLinks) {
        g_axis_max_velocity[kIdxX2] = v;
        g_axis_max_velocity_valid[kIdxX2] = true;
      }
      refreshPersistFromRuntime();
      persistSaveV4();
      const char* tag = (kind == AxisTokenKind::VirtualX) ? "x" : "p";
      consolePrintfAxis(tag, "-> maxvelocity %.1f sps\n", getVirtualAxisMaxVelocity(
                        (kind == AxisTokenKind::VirtualX) ? VirtualAxis::X : VirtualAxis::P));
      if (ceiling_clamped) {
        consolePrintfAxis(tag, "! capped to global ceiling %.1f sps\n", kMaxVelocityCeiling);
      }
    }
  } else {
    g_config.max_velocity = v;
    saveConfig();
    consolePrintf("-> maxvelocity %.1f sps\n", min(g_config.max_velocity, kMaxVelocityCeiling));
    if (ceiling_clamped) {
      consolePrintf("! capped to global ceiling %.1f sps\n", kMaxVelocityCeiling);
    }
  }
}

static void cmdMaxAccel(char* save) {
  char* savep = save;
  char* t1 = nextToken(savep);
  if (!t1) {
    consolePrintf("maxaccel: %.1f sps^2\n", g_config.max_accel);
    return;
  }
  size_t axis_idx = 0;
  AxisTokenKind kind = AxisTokenKind::None;
  bool has_axis = parseAxisToken(t1, kind, axis_idx);
  char* value_token = has_axis ? nextToken(savep) : t1;
  if (!value_token) {
    if (has_axis) {
      if (kind == AxisTokenKind::Physical) {
        consolePrintfAxis(links[axis_idx].name, "maxaccel: %.1f sps^2\n", getAxisMaxAccel(axis_idx));
      } else if (kind == AxisTokenKind::VirtualX) {
        consolePrintfAxis("x", "maxaccel: %.1f sps^2\n", getVirtualAxisMaxAccel(VirtualAxis::X));
      } else if (kind == AxisTokenKind::VirtualP) {
        consolePrintfAxis("p", "maxaccel: %.1f sps^2\n", getVirtualAxisMaxAccel(VirtualAxis::P));
      }
    } else {
      consolePrintf("maxaccel: %.1f sps^2\n", g_config.max_accel);
    }
    return;
  }

  float a = strtof(value_token, nullptr);
  if (!isfinite(a) || a <= 0.0f) {
    if (has_axis) {
      consolePrintfAxis((kind == AxisTokenKind::VirtualX) ? "x" :
                        (kind == AxisTokenKind::VirtualP) ? "p" : links[axis_idx].name,
                        "! maxaccel must be >0\n");
    } else {
      consolePrintln("! maxaccel must be >0");
    }
    return;
  }

  if (has_axis) {
    if (kind == AxisTokenKind::Physical) {
      if (axis_idx < kMaxPersistLinks) {
        g_axis_max_accel[axis_idx] = a;
        g_axis_max_accel_valid[axis_idx] = true;
        refreshPersistFromRuntime();
        persistSaveV4();
      }
      consolePrintfAxis(links[axis_idx].name, "-> maxaccel %.1f sps^2\n", getAxisMaxAccel(axis_idx));
    } else {
      size_t v_idx = (kind == AxisTokenKind::VirtualX) ? 0u : 1u;
      g_virtual_max_accel[v_idx] = a;
      g_virtual_max_accel_valid[v_idx] = true;
      if (kIdxX1 < kMaxPersistLinks) {
        g_axis_max_accel[kIdxX1] = a;
        g_axis_max_accel_valid[kIdxX1] = true;
      }
      if (kIdxX2 < kMaxPersistLinks) {
        g_axis_max_accel[kIdxX2] = a;
        g_axis_max_accel_valid[kIdxX2] = true;
      }
      refreshPersistFromRuntime();
      persistSaveV4();
      const char* tag = (kind == AxisTokenKind::VirtualX) ? "x" : "p";
      consolePrintfAxis(tag, "-> maxaccel %.1f sps^2\n", getVirtualAxisMaxAccel(
                        (kind == AxisTokenKind::VirtualX) ? VirtualAxis::X : VirtualAxis::P));
    }
  } else {
    g_config.max_accel = a;
    saveConfig();
    consolePrintf("-> maxaccel %.1f sps^2\n", g_config.max_accel);
  }
}

static void cmdShowConfig(char*) { printConfig(); }

static void cmdLed(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t link_idx = 0;
  if (!axis || !findLinkIndexByName(axis, link_idx)) {
    consolePrintln("Usage: led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]");
    return;
  }

  char* led_tokens[kLedFrameLedCount] = {};
  for (size_t i = 0; i < kLedFrameLedCount; ++i) {
    led_tokens[i] = nextToken(savep);
    if (!led_tokens[i]) {
      consolePrintln("Usage: led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]");
      return;
    }
  }

  auto parseColorToken = [](const char* tok, uint32_t& rgb, bool& keep) -> bool {
    if (!tok) return false;
    if (strlen(tok) != 6) return false;
    if (!strcmp(tok, "------")) {
      keep = true;
      rgb = 0;
      return true;
    }
    keep = false;
    uint32_t value = 0;
    for (size_t i = 0; i < 6; ++i) {
      char c = tok[i];
      uint8_t nibble = 0;
      if (c >= '0' && c <= '9') nibble = (uint8_t)(c - '0');
      else if (c >= 'a' && c <= 'f') nibble = (uint8_t)(10 + (c - 'a'));
      else if (c >= 'A' && c <= 'F') nibble = (uint8_t)(10 + (c - 'A'));
      else return false;
      value = (value << 4) | nibble;
    }
    rgb = value & 0xFFFFFFu;
    return true;
  };

  auto parseKeyValue = [](const char* tok, char key, long& out) -> bool {
    if (!tok || strlen(tok) < 3) return false;
    if (toupper((unsigned char)tok[0]) != key || tok[1] != '=') return false;
    char* end = nullptr;
    long val = strtol(tok + 2, &end, 10);
    if (!end || *end != '\0') return false;
    out = val;
    return true;
  };

  uint32_t rgb[kLedFrameLedCount] = {};
  uint8_t mask = 0;
  for (size_t i = 0; i < kLedFrameLedCount; ++i) {
    bool keep = false;
    uint32_t value = 0;
    if (!parseColorToken(led_tokens[i], value, keep)) {
      consolePrintln("Usage: led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]");
      return;
    }
    if (!keep) {
      mask |= (uint8_t)(1u << i);
      rgb[i] = value;
    }
  }

  uint32_t transition_ms = 0;
  bool has_brightness = false;
  int brightness = 0;

  char* opt = nullptr;
  while ((opt = nextToken(savep)) != nullptr) {
    long value = 0;
    if (parseKeyValue(opt, 'T', value)) {
      if (value < 0) {
        consolePrintln("Usage: led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]");
        return;
      }
      transition_ms = (uint32_t)value;
    } else if (parseKeyValue(opt, 'B', value)) {
      if (value < 0) value = 0;
      if (value > 255) value = 255;
      brightness = (int)value;
      has_brightness = true;
    } else {
      consolePrintln("Usage: led <axis> <led0> <led1> <led2> <led3> <led4> <led5> <led6> <led7> [T=<ms>] [B=<0-255>]");
      return;
    }
  }

  LedFrame frame{};
  frame.cmd = (uint8_t)CommandId::SetPixels;
  frame.flags = has_brightness ? kLedFrameFlagBrightness : 0u;
  frame.brightness = has_brightness ? (uint8_t)brightness : 0u;
  frame.mask = mask;
  frame.transition_ms = transition_ms;
  for (size_t i = 0; i < kLedFrameLedCount; ++i) {
    frame.rgb[i][0] = (uint8_t)((rgb[i] >> 16) & 0xFF);
    frame.rgb[i][1] = (uint8_t)((rgb[i] >> 8) & 0xFF);
    frame.rgb[i][2] = (uint8_t)(rgb[i] & 0xFF);
  }
  sendCommandPayload(*links[link_idx].port, reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
}

static void cmdMeasure(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  char* t1 = nextToken(savep);
  size_t idx = 0;
  if (!axis || !t1 || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: measure <axis> <duration_s>");
    return;
  }
  if (!equalsIgnoreCase(links[idx].name, "r")) {
    consolePrintfAxis(links[idx].name, "! measure only supported on axis R\n");
    return;
  }
  float duration_s = strtof(t1, nullptr);
  if (!isfinite(duration_s) || duration_s <= 0.0f) {
    consolePrintln("Usage: measure <axis> <duration_s>");
    return;
  }
  CommandFrame c{};
  c.axis_id = 0;
  c.cmd = (uint8_t)CommandId::MeasureRange;
  c.p0 = duration_s;
  sendCommand(*links[idx].port, c);
  consolePrintfAxis(links[idx].name, "-> measure %.2fs\n", duration_s);
}

static void cmdHi(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: hi <axis>");
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::DisplayHello);
}

static void cmdReboot(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: reboot <axis>");
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::Reboot);
  consolePrintfAxis(links[idx].name, "-> reboot\n");
}

static void cmdTmcSettings(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: tmcsettings <axis>");
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::TmcSettings);
}

static void cmdTmcStatus(char* save) {
  char* savep = save;
  char* axis = nextToken(savep);
  size_t idx = 0;
  if (!axis || !findLinkIndexByName(axis, idx)) {
    consolePrintln("Usage: tmcstatus <axis>");
    return;
  }
  sendCmdTo(idx, (uint8_t)CommandId::TmcStatus);
}

struct ConsoleCommand {
  const char* name;
  ConsoleCmdHandler handler;
};

static const ConsoleCommand kConsoleCommands[] = {
  { "help",          cmdHelp },
  { "?",             cmdHelp },
  { "en",            cmdEnable },
  { "dis",           cmdDisable },
  { "driverstatus",  cmdDriverStatus },
  { "driversettings",cmdDriverSettings },
  { "setname",       cmdSetName },
  { "cur",           cmdCurrent },
  { "ms",            cmdMicrosteps },
  { "move",          cmdMove },
  { "moveto",        cmdMoveTo },
  { "pos",           cmdPos },
  { "moveabs",       cmdMoveAbs },
  { "home",          cmdHome },
  { "coordstatus",   cmdCoordStatus },
  { "stop",          cmdStop },
  { "standstillmode",cmdStandstillMode },
  { "maxvelocity",   cmdMaxVelocity },
  { "maxaccel",      cmdMaxAccel },
  { "showconfig",    cmdShowConfig },
  { "led",           cmdLed },
  { "measure",       cmdMeasure },
  { "hi",            cmdHi },
  { "reboot",        cmdReboot },
  { "tmcsettings",   cmdTmcSettings },
  { "tmcstatus",     cmdTmcStatus },
};

static void handleLine(const char* ln) {
  char buf[128];
  strncpy(buf, ln, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char* save = nullptr;
  char* t0 = strtok_r(buf, kCmdDelims, &save);
  if (!t0) return;
  for (char* p = t0; *p; ++p) *p = tolower(*p);

  for (const auto& cmd : kConsoleCommands) {
    if (!strcmp(t0, cmd.name)) {
      cmd.handler(save);
      return;
    }
  }

  consolePrintf("Unknown command: %s\n", t0);
}

static void serviceConsoleInput(Stream& s, char* buf, size_t& pos, size_t cap) {
  while (s.available()) {
    int c = s.read();
    if (c < 0) break;
    if (c == '\r') continue;
    if (c == '\n') {
      if (pos > 0) {
        buf[pos] = '\0';
        handleLine(buf);
      }
      pos = 0;
    } else if (c == 0x08 || c == 0x7F) {
      if (pos > 0) --pos;
    } else if (pos + 1 < cap) {
      buf[pos++] = (char)c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  delay(50);

  piConsole.begin(1000000);

  pinMode(kStopButtonPin, INPUT_PULLUP);
  g_stop_button_prev_pressed = (digitalRead(kStopButtonPin) == LOW);

  loadPersistAll();

  for (size_t i = 0; i < kNumLinks; ++i) {
    links[i].port->begin(1000000);
    if (i < (sizeof(kDefaultLinkNames) / sizeof(kDefaultLinkNames[0]))) {
      setLinkName(i, kDefaultLinkNames[i]);
    }
    links[i].last_ms = 0;
    links[i].was_connected = false;
    links[i].motion = MotionState{};
    links[i].synth.reset();
    resetVelocityEstimate(links[i]);
  }

  consolePrintln("Teensy41 link ready @1Mbaud on Serial8");
  printConfig();
  printHelp();
}

void loop() {
  handleStopButton();

  for (size_t i = 0; i < kNumLinks; ++i) {
    processLinkInput(i);
  }

  serviceConsoleInput(Serial, linebuf, linepos, sizeof(linebuf));
  serviceConsoleInput(piConsole, pi_linebuf, pi_linepos, sizeof(pi_linebuf));

  uint32_t now_ms = millis();
  serviceJogUntilLimit(now_ms);
  serviceHomeSequence(now_ms);
  serviceCoordMoves(now_ms);
  printLinksIfDue(now_ms);
  sendPingsIfDue(now_ms);
}
