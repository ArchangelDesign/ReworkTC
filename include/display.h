/**
 * @file display.h
 * @brief OLED display interface for ReworkTC
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

#pragma once

#include <Arduino.h>

#define SCREEN_ADDRESS 0x3C 

#if DISABLE_DISPLAY==1
// Display disabled - provide stub functions
void display_init() {
  Serial.println("Display disabled by build flag");
}

void display_print_text(int16_t x, int16_t y, const char* text, uint8_t size = 1) {
  // No-op when display disabled
}

void display_error(const char* errorMsg) {
  Serial.println(errorMsg);
}

void display_status(float temperature, int pid_current_power, bool pid_enabled, int pid_setpoint) {
  // No-op when display disabled
}

#else
// Display enabled - include libraries and implement full functionality
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

extern float pid_kp;
extern float pid_ki;
extern float pid_kd;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool display_initialized = false;

void display_refresh() {
  if (!display_initialized) return;
  display.display();
}

void display_clear() {
  if (!display_initialized) return;
  display.clearDisplay();
}

void display_init() {
  // Power on the OLED display (Heltec boards require this)
  if (VEXT_CTRL != -1) {
    pinMode(VEXT_CTRL, OUTPUT);
    digitalWrite(VEXT_CTRL, LOW); // LOW = power ON for Heltec boards
    delay(50);
  }
  
  // Initialize I2C with custom or default pins
  Wire.begin(OLED_SDA, OLED_SCL);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }
  display_initialized = true;
  
  // Flip display if flag is set (for boards mounted upside down)
  #ifdef FLIP_DISPLAY
  #if FLIP_DISPLAY==1
  display.setRotation(2);  // 180 degree rotation
  #endif
  #endif
  
  display_clear();
  display_refresh();
}

void display_print_text(int16_t x, int16_t y, const char* text, uint8_t size = 1) {
  if (!display_initialized) {
    return;    
  }
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(text);
}

void display_error(const char* errorMsg) {
  if (!display_initialized) return;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(errorMsg);
}

void display_status(float temperature, int pid_current_power, bool pid_enabled, int pid_setpoint) {
  if (!display_initialized) return;

  char tempStr[20];
  snprintf(tempStr, sizeof(tempStr), "%.1fC", temperature);
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(tempStr);

  // Draw power as a bar using '=' signs
  display.setTextSize(1);
  display.setCursor(0, 30);
  int barLength = map(pid_current_power, 0, 100, 0, 18); // 18 chars max
  char bar[20];
  for (int i = 0; i < barLength; ++i) bar[i] = '=';
  for (int i = barLength; i < 18; ++i) bar[i] = ' ';
  bar[18] = ']';
  bar[19] = '\0';
  display.print('[');
  display.println(bar);
  
  display.setCursor(0, 42);
  display.print(pid_kp);
  display.print("  ");
  display.print(pid_ki);
  display.print("  ");
  display.print(pid_kd);

  if (pid_enabled) {
    char setpointStr[20];
    snprintf(setpointStr, sizeof(setpointStr), "Set: %dC", pid_setpoint);
    display.setCursor(0, 54);
    display.println(setpointStr);
  }
}


#endif  // DISABLE_DISPLAY