#include "anna_ui.h"

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL6180X.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "anna_debug.h"
#include "anna_driver.h"
#include "anna_link.h"
#include "anna_pins.h"
#include "anna_state.h"

namespace {
Adafruit_SSD1306 g_display(128, 64, &Wire, -1);
Adafruit_NeoPixel g_leds(kNeoPixelCount, PIN_NEOPIX, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel g_measure_led(1, PIN_R_NEOPIX, NEO_GRB + NEO_KHZ800);
Adafruit_VL6180X g_vl6180x;

TaskHandle_t g_ui_task = nullptr;

static constexpr uint32_t kUiUpdateMs  = 20;
static constexpr uint32_t kLedUpdateMs = 50;
static constexpr uint32_t kLedCommandTimeoutMs = 5000;
static constexpr uint16_t kLinkLedIndex = 0;
static constexpr uint32_t kMeasureSampleMs = 20;

static constexpr size_t kDeviceNameMaxLen = 8;
char g_device_name[kDeviceNameMaxLen + 1] = "???";

char g_msg_text[64] = {};
uint32_t g_msg_until_ms = 0;

uint32_t g_stop_sweep_until_ms = 0;

uint32_t g_pixel_override[kNeoPixelCount] = {}; // raw RGB (RRGGBB)
bool     g_pixel_override_set[kNeoPixelCount] = {};
bool     g_led_dirty = true;
uint8_t  g_led_user_brightness = 255;
uint32_t g_last_led_cmd_ms = 0;

bool     g_led_fade_active = false;
uint32_t g_led_fade_start_ms = 0;
uint32_t g_led_fade_duration_ms = 0;
uint8_t  g_led_fade_mask = 0;
uint32_t g_led_fade_start[kNeoPixelCount] = {};
uint32_t g_led_fade_target[kNeoPixelCount] = {};

uint32_t g_last_led_out[kNeoPixelCount] = {};
bool     g_last_led_out_valid = false;

portMUX_TYPE g_ui_mux = portMUX_INITIALIZER_UNLOCKED;

bool g_measure_axis_r = false;
bool g_measure_hw_pending = false;
bool g_measure_hw_ready = false;
bool g_measure_request_pending = false;
uint32_t g_measure_request_ms = 0;

static inline uint8_t scale8(uint8_t v, uint8_t br) {
  return (uint8_t)((uint16_t)v * (uint16_t)br / 255u);
}

static inline uint32_t colorScaled(uint8_t r, uint8_t g, uint8_t b, uint8_t br) {
  return g_leds.Color(scale8(r, br), scale8(g, br), scale8(b, br));
}

static inline uint32_t scaleRgb(uint32_t rgb, uint8_t brightness) {
  uint8_t r = (uint8_t)((rgb >> 16) & 0xFF);
  uint8_t g = (uint8_t)((rgb >> 8) & 0xFF);
  uint8_t b = (uint8_t)(rgb & 0xFF);
  return colorScaled(r, g, b, brightness);
}

static inline uint8_t lerp8(uint8_t a, uint8_t b, uint8_t t) {
  int16_t delta = (int16_t)b - (int16_t)a;
  int16_t scaled = (int16_t)((int32_t)delta * (int32_t)t / 255);
  int16_t out = (int16_t)a + scaled;
  if (out < 0) out = 0;
  if (out > 255) out = 255;
  return (uint8_t)out;
}

static inline uint32_t lerpColor(uint32_t a, uint32_t b, uint8_t t) {
  uint8_t a0 = (uint8_t)((a >> 16) & 0xFF);
  uint8_t a1 = (uint8_t)((a >> 8) & 0xFF);
  uint8_t a2 = (uint8_t)(a & 0xFF);
  uint8_t b0 = (uint8_t)((b >> 16) & 0xFF);
  uint8_t b1 = (uint8_t)((b >> 8) & 0xFF);
  uint8_t b2 = (uint8_t)(b & 0xFF);
  uint32_t out = ((uint32_t)lerp8(a0, b0, t) << 16) |
                 ((uint32_t)lerp8(a1, b1, t) << 8) |
                 (uint32_t)lerp8(a2, b2, t);
  return out;
}

static inline uint16_t linkLedIndex() {
  if (kNeoPixelCount == 0) return 0;
  if (kLinkLedIndex < kNeoPixelCount) return kLinkLedIndex;
  return (uint16_t)(kNeoPixelCount - 1);
}

static void drawDeviceName(const char* name) {
  if (!name) return;
  size_t len = strlen(name);
  if (len == 0) return;
  int16_t x = 128 - (int16_t)(len * 6);
  if (x < 0) x = 0;
  g_display.setTextSize(1);
  g_display.setCursor(x, 56);
  g_display.print(name);
}

static void drawUi(const AxisStateSnapshot& s,
                   int32_t rp_steps,
                   const char* device_name,
                   const char* msg_text,
                   uint32_t msg_until_ms,
                   uint32_t now_ms) {
  // Overlay message, if active.
  if (msg_text && (int32_t)(now_ms - msg_until_ms) < 0) {
    g_display.clearDisplay();
    g_display.setTextSize(2);
    g_display.setCursor(0, 0);
    g_display.println(msg_text);
    drawDeviceName(device_name);
    g_display.display();
    return;
  }

  g_display.clearDisplay();

  // Line 1: limit switch + (optional) temperature.
  g_display.setTextSize(1);
  g_display.setCursor(0, 0);
  g_display.print("SW:");
  g_display.print(s.limit_pressed ? "ON" : "OFF");

  if (s.temp_ok && isfinite(s.temp_c) && s.temp_c > -55.0f && s.temp_c < 125.0f) {
    int16_t t10 = (int16_t)lrintf(s.temp_c * 10.0f);
    bool neg = (t10 < 0);
    if (neg) t10 = (int16_t)(-t10);
    int16_t tw = t10 / 10;
    int16_t tf = t10 % 10;
    char temp_buf[16];
    if (neg) snprintf(temp_buf, sizeof(temp_buf), "T:-%d.%dC", (int)tw, (int)tf);
    else     snprintf(temp_buf, sizeof(temp_buf), "T:%d.%dC", (int)tw, (int)tf);
    int16_t x = 128 - (int16_t)(strlen(temp_buf) * 6);
    if (x < 0) x = 0;
    g_display.setCursor(x, 0);
    g_display.print(temp_buf);
  }

  // Line 2: absolute position (AP, unwrapped degrees).
  if (s.encoder_ok && isfinite(s.angle_deg_unwrapped)) {
    g_display.setTextSize(2);
    g_display.setCursor(0, 14);
    int32_t p = (int32_t)lrintf(s.angle_deg_unwrapped);
    g_display.printf("AP:%ld", (long)p);
  }

  // Line 3: relative position (RP, full steps from boot / last limit hit).
  g_display.setTextSize(2);
  g_display.setCursor(0, 34);
  g_display.printf("RP:%ld", (long)rp_steps);

  drawDeviceName(device_name);
  g_display.display();
}

static void drawMeasure(int32_t mm, bool valid) {
  g_display.clearDisplay();
  if (valid) {
    char buf[8];
    snprintf(buf, sizeof(buf), "%ld", (long)mm);
    g_display.setTextSize(4);
    int16_t x = (128 - (int16_t)(strlen(buf) * 6 * 4)) / 2;
    if (x < 0) x = 0;
    int16_t y = (64 - (int16_t)(8 * 4)) / 2;
    if (y < 0) y = 0;
    g_display.setCursor(x, y);
    g_display.print(buf);
  } else {
    g_display.setTextSize(3);
    int16_t x = (128 - (int16_t)(3 * 6 * 3)) / 2;
    int16_t y = (64 - (int16_t)(8 * 3)) / 2;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    g_display.setCursor(x, y);
    g_display.print("---");
  }
  g_display.display();
}

static void reportRangeError(uint8_t err, uint32_t now_ms, uint8_t& last_err, uint32_t& last_err_ms) {
  if (err == 0) return;
  if (err != last_err || (uint32_t)(now_ms - last_err_ms) >= 1000) {
    char msg[32];
    snprintf(msg, sizeof(msg), "range.err=%u", (unsigned)err);
    annaLinkSendEventText(msg);
    last_err = err;
    last_err_ms = now_ms;
  }
}

static uint32_t rainbowColor(uint32_t now_ms) {
  uint16_t hue = (uint16_t)((now_ms * 8u) & 0xFFFFu);
  return g_measure_led.gamma32(g_measure_led.ColorHSV(hue));
}

static bool computeLeds(const AxisStateSnapshot& s,
                        const uint32_t pixel_override[kNeoPixelCount],
                        const bool pixel_override_set[kNeoPixelCount],
                        uint8_t user_brightness,
                        uint32_t stop_sweep_until_ms,
                        uint32_t now_ms,
                        uint32_t out[kNeoPixelCount]) {
  // Stop alert overrides everything: sweep a red pixel across the strip.
  if ((int32_t)(stop_sweep_until_ms - now_ms) > 0) {
    for (uint16_t i = 0; i < kNeoPixelCount; ++i) out[i] = 0;
    if (kNeoPixelCount > 0) {
      const uint16_t span = (kNeoPixelCount > 1) ? (uint16_t)(2 * (kNeoPixelCount - 1)) : 1;
      uint16_t step = (uint16_t)((now_ms / 80) % span);
      if (step >= kNeoPixelCount) step = (uint16_t)(span - step);
      out[step] = colorScaled(255, 0, 0, 180);
    }
    return true;
  }

  for (uint16_t i = 0; i < kNeoPixelCount; ++i) out[i] = 0;

  // Default UI pixels.
  const uint16_t link_idx = linkLedIndex();
  if (kNeoPixelCount > 0) {
    out[link_idx] = s.link_connected ? colorScaled(0, 255, 0, 127) : colorScaled(255, 0, 0, 127);
  }
  out[1] = s.limit_pressed ? colorScaled(0, 128, 255, 127) : 0;

  // Apply user overrides.
  for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
    if (pixel_override_set[i]) out[i] = scaleRgb(pixel_override[i], user_brightness);
  }
  return false;
}

#if SCANBOT_DEBUG
static void i2cScan() {
  Serial.println("I2C scan (shared bus):");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) Serial.printf("0x%02X\n", addr);
  }
}

static void i2cScanToDisplay(uint32_t hold_ms) {
  g_display.clearDisplay();
  g_display.setTextSize(1);
  g_display.setCursor(0, 0);
  g_display.print("I2C0:");
  const int line_h = 8;
  const int top_y  = 10;
  const int col_w  = 42;
  int row = 0;
  int col = 0;
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      int x = col * col_w;
      int y = top_y + row * line_h;
      if (y + line_h > 64) {
        col++;
        row = 0;
        x = col * col_w;
        y = top_y;
      }
      if (x < 128) {
        g_display.setCursor(x, y);
        g_display.printf("0x%02X", addr);
        row++;
        count++;
      }
    }
  }
  if (count == 0) {
    g_display.setCursor(0, top_y);
    g_display.print("(none)");
  }
  g_display.display();
  delay(hold_ms);
}
#endif

} // namespace

void annaUiInit() {
  // OLED init - prevent Wire from being re-begun on default pins.
  g_display.begin(SSD1306_SWITCHCAPVCC, 0x3C, -1, false);
  g_display.clearDisplay();
  g_display.setTextSize(2);
  g_display.setTextColor(SSD1306_WHITE);
  g_display.display();

#if SCANBOT_DEBUG
  i2cScan();
  i2cScanToDisplay(2000);
  g_display.clearDisplay();
  g_display.display();
#endif

  g_leds.begin();
  g_leds.setBrightness(40);
  g_leds.fill(0);
  g_leds.show();
  portENTER_CRITICAL(&g_ui_mux);
  memset(g_last_led_out, 0, sizeof(g_last_led_out));
  g_last_led_out_valid = true;
  portEXIT_CRITICAL(&g_ui_mux);
}

void annaUiNotify() {
  if (!g_ui_task) return;
  xTaskNotifyGive(g_ui_task);
}

void annaUiSetDeviceName(const char* name, size_t len) {
  portENTER_CRITICAL(&g_ui_mux);
  memset(g_device_name, 0, sizeof(g_device_name));
  if (!name || len == 0) {
    strncpy(g_device_name, "???", sizeof(g_device_name) - 1);
  } else {
    if (len > kDeviceNameMaxLen) len = kDeviceNameMaxLen;
    size_t written = 0;
    for (; written < len; ++written) {
      char c = name[written];
      if (c == '\0') break;
      if ((uint8_t)c < 32 || (uint8_t)c > 126) c = '?';
      g_device_name[written] = c;
    }
    if (written == 0) strncpy(g_device_name, "???", sizeof(g_device_name) - 1);
    else g_device_name[written] = '\0';
  }
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

void annaUiShowMessage(const char* msg, uint32_t duration_ms) {
  portENTER_CRITICAL(&g_ui_mux);
  if (msg) {
    strncpy(g_msg_text, msg, sizeof(g_msg_text) - 1);
    g_msg_text[sizeof(g_msg_text) - 1] = '\0';
  } else {
    g_msg_text[0] = '\0';
  }
  g_msg_until_ms = millis() + duration_ms;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

void annaUiTriggerStopAlert() {
  portENTER_CRITICAL(&g_ui_mux);
  uint32_t now = millis();
  g_stop_sweep_until_ms = now + 1200;
  strncpy(g_msg_text, "Limit Triggered", sizeof(g_msg_text) - 1);
  g_msg_text[sizeof(g_msg_text) - 1] = '\0';
  g_msg_until_ms = now + 1200;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

void annaUiSetPixelOverride(uint8_t idx, uint32_t rgb, uint8_t brightness) {
  if (idx >= kNeoPixelCount) idx = kNeoPixelCount - 1;
  uint32_t rgb24 = rgb & 0xFFFFFFu;
  uint32_t now_ms = millis();

  portENTER_CRITICAL(&g_ui_mux);
  g_last_led_cmd_ms = now_ms;
  g_led_user_brightness = brightness;
  g_pixel_override[idx] = rgb24;
  g_pixel_override_set[idx] = true;
  g_led_dirty = true;
  g_led_fade_active = false;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

void annaUiSetLedFrame(const LedFrame& frame) {
  const uint8_t mask = frame.mask;
  const bool brightness_present = (frame.flags & kLedFrameFlagBrightness) != 0;
  const uint32_t transition_ms = frame.transition_ms;
  const uint32_t now_ms = millis();

  portENTER_CRITICAL(&g_ui_mux);
  g_last_led_cmd_ms = now_ms;
  if (brightness_present) g_led_user_brightness = frame.brightness;
  const uint8_t user_brightness = g_led_user_brightness;

  for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
    if (mask & (uint8_t)(1u << i)) {
      uint32_t rgb = ((uint32_t)frame.rgb[i][0] << 16) |
                     ((uint32_t)frame.rgb[i][1] << 8) |
                     (uint32_t)frame.rgb[i][2];
      g_pixel_override[i] = rgb;
      g_pixel_override_set[i] = true;
    }
  }

  uint8_t fade_mask = 0;
  if (transition_ms > 0) {
    fade_mask |= mask;
    if (brightness_present) {
      for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
        if (g_pixel_override_set[i]) fade_mask |= (uint8_t)(1u << i);
      }
    }
    if (fade_mask != 0) {
      g_led_fade_active = true;
      g_led_fade_start_ms = now_ms;
      g_led_fade_duration_ms = transition_ms;
      g_led_fade_mask = fade_mask;
      for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
        if (fade_mask & (uint8_t)(1u << i)) {
          g_led_fade_start[i] = g_last_led_out_valid ? g_last_led_out[i] : 0;
          if (g_pixel_override_set[i]) g_led_fade_target[i] = scaleRgb(g_pixel_override[i], user_brightness);
          else g_led_fade_target[i] = g_led_fade_start[i];
        }
      }
    } else {
      g_led_fade_active = false;
    }
  } else {
    g_led_fade_active = false;
  }

  g_led_dirty = true;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

bool g_measure_active = false;
bool g_measure_sensor_powered = false;
bool g_measure_sensor_ok = false;
uint32_t g_measure_end_ms = 0;
uint32_t g_measure_next_sample_ms = 0;
int32_t g_measure_last_drawn = INT32_MIN;
bool g_measure_last_drawn_valid = false;
uint8_t g_measure_last_err = 0;
uint32_t g_measure_last_err_ms = 0;

static bool startMeasurement(uint32_t duration_ms, uint32_t now_ms) {
  if (!g_measure_hw_ready || duration_ms == 0) return false;
  if (g_measure_active && g_measure_sensor_ok) {
    g_measure_end_ms = now_ms + duration_ms;
    g_measure_next_sample_ms = now_ms;
    g_measure_last_drawn = INT32_MIN;
    g_measure_last_drawn_valid = false;
    g_measure_last_err = 0;
    g_measure_last_err_ms = 0;
    axisStateSetRange(NAN, false);
    return true;
  }
  if (!g_measure_sensor_powered) {
    pinMode(PIN_R_VL6180X_SHUT, OUTPUT);
    digitalWrite(PIN_R_VL6180X_SHUT, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    g_measure_sensor_powered = true;
    g_measure_sensor_ok = false;
  }
  if (!g_measure_sensor_ok) {
    g_measure_sensor_ok = g_vl6180x.begin(&Wire);
    if (!g_measure_sensor_ok) {
      annaLinkSendEventText("measure.reject: vl6180x init failed");
      digitalWrite(PIN_R_VL6180X_SHUT, LOW);
      g_measure_sensor_powered = false;
      axisStateSetRange(NAN, false);
      return false;
    }
  }
  g_measure_active = true;
  g_measure_end_ms = now_ms + duration_ms;
  g_measure_next_sample_ms = now_ms;
  g_measure_last_drawn = INT32_MIN;
  g_measure_last_drawn_valid = false;
  g_measure_last_err = 0;
  g_measure_last_err_ms = 0;
  axisStateSetRange(NAN, false);
  return true;
}

static void stopMeasurement() {
  g_measure_active = false;
  g_measure_sensor_ok = false;
  if (g_measure_sensor_powered) {
    digitalWrite(PIN_R_VL6180X_SHUT, LOW);
    g_measure_sensor_powered = false;
  }
  axisStateSetRange(NAN, false);
  if (g_measure_hw_ready) {
    g_measure_led.fill(0);
    g_measure_led.show();
  }
  g_measure_last_drawn = INT32_MIN;
  g_measure_last_drawn_valid = false;
}

void annaUiTask(void*) {
  g_ui_task = xTaskGetCurrentTaskHandle();

  uint32_t last_led_ms = 0;
  uint32_t last_led_out[kNeoPixelCount] = {};
  uint32_t last_measure_led = 0;

  for (;;) {
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kUiUpdateMs));

    uint32_t now_ms = millis();
    AxisStateSnapshot s = axisStateRead(now_ms);
    int32_t rp_steps = annaDriverRelativePositionSteps();

    // Snapshot UI control state (message/device name/overrides) without holding a lock during I2C.
    char device_name[kDeviceNameMaxLen + 1];
    char msg_text[sizeof(g_msg_text)];
    uint32_t msg_until_ms = 0;
    uint32_t stop_sweep_until_ms = 0;
    uint32_t pixel_override[kNeoPixelCount];
    bool pixel_override_set[kNeoPixelCount];
    uint8_t user_brightness = 255;
    bool led_fade_active = false;
    uint32_t led_fade_start_ms = 0;
    uint32_t led_fade_duration_ms = 0;
    uint8_t led_fade_mask = 0;
    uint32_t led_fade_start[kNeoPixelCount];
    uint32_t led_fade_target[kNeoPixelCount];
    bool led_dirty = false;
    bool measure_hw_pending = false;
    bool measure_req_pending = false;
    uint32_t measure_req_ms = 0;
    bool measure_axis_r = false;
    portENTER_CRITICAL(&g_ui_mux);
    const uint16_t link_idx = linkLedIndex();
    if (kNeoPixelCount > 0 && (int32_t)(now_ms - g_last_led_cmd_ms) >= (int32_t)kLedCommandTimeoutMs) {
      if (g_pixel_override_set[link_idx]) {
        g_pixel_override_set[link_idx] = false;
        g_led_dirty = true;
      }
      const uint8_t link_bit = (uint8_t)(1u << link_idx);
      if (g_led_fade_active && (g_led_fade_mask & link_bit)) {
        g_led_fade_mask = (uint8_t)(g_led_fade_mask & ~link_bit);
        if (g_led_fade_mask == 0) g_led_fade_active = false;
      }
    }
    strncpy(device_name, g_device_name, sizeof(device_name));
    device_name[sizeof(device_name) - 1] = '\0';
    strncpy(msg_text, g_msg_text, sizeof(msg_text));
    msg_text[sizeof(msg_text) - 1] = '\0';
    msg_until_ms = g_msg_until_ms;
    stop_sweep_until_ms = g_stop_sweep_until_ms;
    memcpy(pixel_override, g_pixel_override, sizeof(pixel_override));
    memcpy(pixel_override_set, g_pixel_override_set, sizeof(pixel_override_set));
    user_brightness = g_led_user_brightness;
    led_fade_active = g_led_fade_active;
    led_fade_start_ms = g_led_fade_start_ms;
    led_fade_duration_ms = g_led_fade_duration_ms;
    led_fade_mask = g_led_fade_mask;
    memcpy(led_fade_start, g_led_fade_start, sizeof(led_fade_start));
    memcpy(led_fade_target, g_led_fade_target, sizeof(led_fade_target));
    led_dirty = g_led_dirty;
    g_led_dirty = false;
    measure_hw_pending = g_measure_hw_pending;
    if (measure_hw_pending) g_measure_hw_pending = false;
    measure_req_pending = g_measure_request_pending;
    if (measure_req_pending) {
      measure_req_ms = g_measure_request_ms;
      g_measure_request_pending = false;
    }
    measure_axis_r = g_measure_axis_r;
    portEXIT_CRITICAL(&g_ui_mux);

    if (measure_hw_pending && measure_axis_r) {
      pinMode(PIN_R_VL6180X_SHUT, OUTPUT);
      digitalWrite(PIN_R_VL6180X_SHUT, LOW);
      g_measure_led.begin();
      g_measure_led.setBrightness(40);
      g_measure_led.fill(0);
      g_measure_led.show();
      g_measure_hw_ready = true;
    }

    if (measure_req_pending) {
      if (!measure_axis_r) {
        annaLinkSendEventText("measure.reject: axis not R");
      } else if (!g_measure_hw_ready) {
        annaLinkSendEventText("measure.reject: hw not ready");
      } else {
        (void)startMeasurement(measure_req_ms, now_ms);
      }
    }

    if (g_measure_active) {
      if ((int32_t)(now_ms - g_measure_end_ms) >= 0) {
        stopMeasurement();
      } else if ((int32_t)(now_ms - g_measure_next_sample_ms) >= 0) {
        g_measure_next_sample_ms = now_ms + kMeasureSampleMs;
        uint8_t range = g_vl6180x.readRange();
        uint8_t status = g_vl6180x.readRangeStatus();
        if (status == VL6180X_ERROR_NONE) {
          axisStateSetRange((float)range, true);
          char msg[32];
          snprintf(msg, sizeof(msg), "range_mm:%0.1f", (float)range);
          annaLinkSendEventText(msg);
          if (!g_measure_last_drawn_valid || (int32_t)range != g_measure_last_drawn) {
            g_measure_last_drawn = (int32_t)range;
            g_measure_last_drawn_valid = true;
            drawMeasure(g_measure_last_drawn, true);
          }
        } else {
          axisStateSetRange(NAN, false);
          reportRangeError(status, now_ms, g_measure_last_err, g_measure_last_err_ms);
          if (g_measure_last_drawn_valid || g_measure_last_drawn == INT32_MIN) {
            g_measure_last_drawn_valid = false;
            g_measure_last_drawn = INT32_MIN;
            drawMeasure(0, false);
          }
        }
      }
    }

    if (!g_measure_active) {
      drawUi(s, rp_steps, device_name, msg_text, msg_until_ms, now_ms);
    }

    // LEDs: 20 Hz (or dirty).
    if ((uint32_t)(now_ms - last_led_ms) >= kLedUpdateMs) {
      last_led_ms = now_ms;
      uint32_t out[kNeoPixelCount];
      bool stop_override = computeLeds(s,
                                       pixel_override,
                                       pixel_override_set,
                                       user_brightness,
                                       stop_sweep_until_ms,
                                       now_ms,
                                       out);

      bool clear_fade = false;
      if (led_fade_active && !stop_override) {
        uint8_t t = 0;
        if (led_fade_duration_ms > 0) {
          uint32_t elapsed = now_ms - led_fade_start_ms;
          if (elapsed >= led_fade_duration_ms) {
            t = 255;
            clear_fade = true;
          } else {
            t = (uint8_t)(((uint64_t)elapsed * 255u) / (uint64_t)led_fade_duration_ms);
          }
        } else {
          t = 255;
          clear_fade = true;
        }
        for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
          if (led_fade_mask & (uint8_t)(1u << i)) {
            out[i] = lerpColor(led_fade_start[i], led_fade_target[i], t);
          }
        }
      }

      bool diff = led_dirty;
      for (uint16_t i = 0; i < kNeoPixelCount && !diff; ++i) {
        if (out[i] != last_led_out[i]) diff = true;
      }
      if (diff) {
        for (uint16_t i = 0; i < kNeoPixelCount; ++i) {
          g_leds.setPixelColor(i, out[i]);
          last_led_out[i] = out[i];
        }
        g_leds.show();
        portENTER_CRITICAL(&g_ui_mux);
        memcpy(g_last_led_out, last_led_out, sizeof(last_led_out));
        g_last_led_out_valid = true;
        portEXIT_CRITICAL(&g_ui_mux);
      } else if (!g_last_led_out_valid) {
        portENTER_CRITICAL(&g_ui_mux);
        memcpy(g_last_led_out, last_led_out, sizeof(last_led_out));
        g_last_led_out_valid = true;
        portEXIT_CRITICAL(&g_ui_mux);
      }
      if (clear_fade) {
        portENTER_CRITICAL(&g_ui_mux);
        g_led_fade_active = false;
        portEXIT_CRITICAL(&g_ui_mux);
      }
    }

    if (g_measure_active && g_measure_hw_ready) {
      uint32_t col = rainbowColor(now_ms);
      if (col != last_measure_led) {
        g_measure_led.setPixelColor(0, col);
        g_measure_led.show();
        last_measure_led = col;
      }
    } else if (last_measure_led != 0 && g_measure_hw_ready) {
      g_measure_led.setPixelColor(0, 0);
      g_measure_led.show();
      last_measure_led = 0;
    }
  }
}

void annaUiConfigureAxisR() {
  portENTER_CRITICAL(&g_ui_mux);
  g_measure_axis_r = true;
  g_measure_hw_pending = true;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
}

bool annaUiRequestMeasure(float duration_s) {
  if (!isfinite(duration_s) || duration_s <= 0.0f) return false;
  portENTER_CRITICAL(&g_ui_mux);
  if (!g_measure_axis_r) {
    portEXIT_CRITICAL(&g_ui_mux);
    return false;
  }
  g_measure_request_ms = (uint32_t)lrintf(duration_s * 1000.0f);
  if (g_measure_request_ms == 0) g_measure_request_ms = 1;
  g_measure_request_pending = true;
  portEXIT_CRITICAL(&g_ui_mux);
  annaUiNotify();
  return true;
}
