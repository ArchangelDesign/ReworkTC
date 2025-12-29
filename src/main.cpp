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

#include <Arduino.h>

#include "display.h"
#include "thermocouple.h"
#include "pid_controller.h"
#include "bt_controller.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  
  display_init();
  display_print_text(0, 0, "Starting...", 2);
  delay(300);
  
  thermocouple_init();
  delay(100);
  
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
  float temperature = thermocouple_read_temperature();
  
  #if DISABLE_DISPLAY!=1
  display.clearDisplay();
  if (isnan(temperature)) {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Error: TC Open");
    display.display();
    Serial.println("Thermocouple error: Open circuit");
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
  if (isnan(temperature)) {
    Serial.println("Thermocouple error: Open circuit");
  } else {
    char tempStr[20];
    snprintf(tempStr, sizeof(tempStr), "%.1fC", temperature);
    const char* status = (pid_enabled && pid_current_power > 0) ? "HEAT: ON" : "HEAT: OFF";
    
    Serial.print("Temp: ");
    Serial.print(tempStr);
    Serial.print(" Power: ");
    Serial.print(pid_current_power);
    Serial.print("% Status: ");
    Serial.println(status);
  }
  #endif
  
  bt_process_commands();
  pid_compute();  // Compute PID output
  pid_update_ssr();  // Update SSR state based on time-proportional control
  delay(250);  // Give MAX6675 time to convert and prevent display flicker
}