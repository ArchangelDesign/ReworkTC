/**
 * @file thermocouple.h
 * @brief MAX6675 thermocouple interface for ReworkTC
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

// MAX6675 pins - can be overridden via build flags
#ifndef MAX6675_CLK
#define MAX6675_CLK  18  // SCK
#endif

#ifndef MAX6675_CS
#define MAX6675_CS   5   // CS
#endif

#ifndef MAX6675_DO
#define MAX6675_DO   19  // MISO (SO)
#endif

int16_t current_temperature_celsius = 0;

void thermocouple_init() {
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_CLK, OUTPUT);
  pinMode(MAX6675_DO, INPUT);
  
  digitalWrite(MAX6675_CS, HIGH);
  digitalWrite(MAX6675_CLK, LOW);
  
  // Give MAX6675 time to stabilize
  delay(100);
}

float thermocouple_read_temperature() {
  uint16_t data = 0;
  
  // Start conversion
  digitalWrite(MAX6675_CS, LOW);
  delayMicroseconds(1);
  
  // Read 16 bits
  for (int i = 15; i >= 0; i--) {
    digitalWrite(MAX6675_CLK, HIGH);
    delayMicroseconds(1);
    
    if (digitalRead(MAX6675_DO)) {
      data |= (1 << i);
    }
    
    digitalWrite(MAX6675_CLK, LOW);
    delayMicroseconds(1);
  }
  
  digitalWrite(MAX6675_CS, HIGH);
  
  // Check for errors
  if (data & 0x4) {
    // Thermocouple open circuit error
    return NAN;
  }
  
  // Extract temperature (bits 15-3)
  data >>= 3;
  
  // Convert to Celsius (0.25°C per bit)
  float temperature = data * 0.25;
  current_temperature_celsius = (int16_t)temperature;
  
  return temperature;
}
