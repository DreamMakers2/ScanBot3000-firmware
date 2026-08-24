// Axis controller: one TMC2209 + AS5600 + SSD1306 + WS2812B-8 + DS18B20 + limit switch.
// Links to teensy41_Beata over UART; multiple instances can run in parallel.

#include <Arduino.h>
#include <Wire.h>

#include "anna_commands.h"
#include "anna_driver.h"
#include "anna_link.h"
#include "anna_pins.h"
#include "anna_state.h"
#include "anna_tasks.h"
#include "anna_ui.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  axisStateInit();

  annaLinkInit();

  // I2C shared bus (OLED)
  Wire.setPins(I2C_SHARED_SDA_PIN, I2C_SHARED_SCL_PIN);
  Wire.begin();
  Wire.setClock(I2C_SHARED_CLOCK_HZ);
  Wire.setTimeOut(50);

  annaUiInit();
  annaDriverInit();
  annaTasksInitPeripherals();

  annaTasksStart();
}

void loop() {
  // All real-time work runs in FreeRTOS tasks (stepper pulses on a dedicated core).
  delay(1000);
}
