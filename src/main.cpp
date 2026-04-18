/**
 * @file main.cpp
 * @brief ReworkTC - Thermocouple-based PID Temperature Controller
 * 
 * @copyright Copyright (c) 2025 Black Horse Repairs LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define REWORKTC_VERSION "1.3.2" 

#include <Arduino.h>
#include <Wire.h>

#include "display.h"
#include "thermocouple.h"
#include "pid_controller.h"
#include "bt_controller.h"
#include "I2C_Anything.h"


#if TOUCH_DISPLAY == 1
#include "touch_interface.h"
#endif

void setup() {
   #if TOUCH_DISPLAY == 1
    #ifndef WIRE_BEGIN_CALLED
      #define WIRE_BEGIN_CALLED
        Wire.begin();
    #endif
  #endif
#ifdef __AVR__
  // Arduino Nano/Uno - use lower baud rate for stability
  // CH340 chip on Nano doesn't support Serial readiness check properly
  
  // Pre-configure UART before enabling to prevent CH340 glitches
  UCSR0A = 0;
  UCSR0B = 0;
  UCSR0C = 0;
  
  Serial.begin(9600);
  delay(500);  // Give CH340 time to stabilize
  Serial.flush();  // Clear any garbage in the buffer
  
  Serial.println();
  Serial.flush();
  delay(100);
#else
  Serial.begin(115200);
  delay(100);
#endif
  
  Serial.println(F("ReworkTC Starting..."));
  
  Serial.println(F("Init display..."));
  display_init();
  display_print_text(0, 0, "Starting...", 2);
  display_refresh();
  delay(300);
  
  Serial.println(F("Init thermocouple..."));
  thermocouple_init();
  delay(100);
  
  Serial.println(F("Init PID..."));
  pid_init();
  delay(100);
  
  #if BT_ENABLED==1
  display_print_text(0, 0, "Init BT...", 1);
  display_refresh();
  delay(200);
  bt_init();
  delay(300);
  #endif
  
  Serial.println(F("ReworkTC Ready"));
}

void loop() {
  static unsigned long lastErrorPrint = 0;
  static bool lastErrorState = false;
  
  float temperature = thermocouple_read_temperature();
  bool hasError = isnan(temperature);
  display_clear();
  if (hasError) {
    display_error("Error: TC Open");
    // Only print error once or every 5 seconds to avoid flooding serial buffer
    if (!lastErrorState || (millis() - lastErrorPrint > 5000)) {
      display_error("Thermocouple error: Open circuit");
      lastErrorPrint = millis();
    }
  } else {
    display_status(temperature, pid_current_power, pid_enabled, pid_setpoint);
  }
  display_refresh();
  
  lastErrorState = hasError;
  bt_process_commands();
  
  // Clear any excess data in serial buffer (prevent overflow)
  #ifdef __AVR__
  if (Serial.available() > 64) {
    while (Serial.available() && Serial.read() != '\n') {
      ; // Clear buffer until newline
    }
  }
  #endif
  
  pid_compute();  // Compute PID output
  pid_update_ssr();  // Update SSR state based on time-proportional control
  delay(250);  // Give MAX6675 time to convert

   #if TOUCH_DISPLAY==1
    send_i2c_Status();
    check_i2c_updates();
  #endif
}