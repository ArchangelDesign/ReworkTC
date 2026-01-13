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

#define REWORKTC_VERSION "1.2.0" 

#include <Arduino.h>

#include "display.h"
#include "thermocouple.h"
#include "pid_controller.h"
#include "bt_controller.h"


void setup() {
  // put your setup code here, to run once:
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
  
  // Keep sending data to keep CH340 "alive" and responsive
  Serial.println();
  Serial.flush();
  delay(100);
#else
  // ESP32 - can handle higher baud rate
  Serial.begin(115200);
  delay(100);
#endif
  
  Serial.println("ReworkTC Starting...");
  
  Serial.println("Init display...");
  display_init();
  display_print_text(0, 0, "Starting...", 2);
  delay(300);
  
  Serial.println("Init thermocouple...");
  thermocouple_init();
  delay(100);
  
  Serial.println("Init PID...");
  pid_init();
  delay(100);
  
  #if BT_ENABLED==1
  display_print_text(0, 0, "Init BT...", 1);
  delay(200);
  bt_init();
  delay(300);
  #endif
  
  #if DISABLE_DISPLAY!=1
  display.clearDisplay();
  #endif
  Serial.println("ReworkTC Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long lastErrorPrint = 0;
  static bool lastErrorState = false;
  
  float temperature = thermocouple_read_temperature();
  bool hasError = isnan(temperature);
  
  #if DISABLE_DISPLAY!=1
  display.clearDisplay();
  if (hasError) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Error: TC Open");
    display.display();
    // Only print error once or every 5 seconds to avoid flooding serial buffer
    if (!lastErrorState || (millis() - lastErrorPrint > 5000)) {
      Serial.println("Thermocouple error: Open circuit");
      lastErrorPrint = millis();
    }
  } else {
    // Display temperature (large)
    char tempStr[20];
    snprintf(tempStr, sizeof(tempStr), "%.1fC", temperature);
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(tempStr);
    
    // Display power percentage
    char powerStr[20];
    snprintf(powerStr, sizeof(powerStr), "Power: %d%%", pid_current_power);
    display.setTextSize(1);
    display.setCursor(0, 30);
    display.println(powerStr);
    
    // Display heater status
    const char* status = (pid_enabled && pid_current_power > 0) ? "HEAT: ON" : "HEAT: OFF";
    display.setCursor(0, 42);
    display.println(status);
    
    // Display setpoint if enabled
    if (pid_enabled) {
      char setpointStr[20];
      snprintf(setpointStr, sizeof(setpointStr), "Set: %dC", pid_setpoint);
      display.setCursor(0, 54);
      display.println(setpointStr);
    }
    
    display.display();
  }
  #else
  // No display - output to serial only
  if (hasError) {
    // Only print error once or every 5 seconds to avoid flooding serial buffer
    if (!lastErrorState || (millis() - lastErrorPrint > 5000)) {
      Serial.println("Thermocouple error: Open circuit");
      lastErrorPrint = millis();
    }
  } else {
    char tempStr[20];
    snprintf(tempStr, sizeof(tempStr), "%.1fC", temperature);
    const char* status = (pid_enabled && pid_current_power > 0) ? "HEAT: ON" : "HEAT: OFF";
    
    // Serial.print("Temp: ");
    // Serial.print(tempStr);
    // Serial.print(" Power: ");
    // Serial.print(pid_current_power);
    // Serial.print("% Status: ");
    // Serial.println(status);
  }
  #endif
  
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
  delay(250);  // Give MAX6675 time to convert and prevent display flicker
}