/**
 * @file pid_controller.h
 * @brief PID temperature controller for ReworkTC
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

// Use Preferences on ESP32, EEPROM on AVR
#ifdef ESP32
#include <Preferences.h>
#else
#include <EEPROM.h>
#endif

// Pin definitions - can be overridden via build flags
#ifndef PID_OUTPUT_PIN
#define PID_OUTPUT_PIN 25
#endif

#ifndef HEATER_LED_PIN
#define HEATER_LED_PIN 22  // Onboard LED for visual indication
#endif

#ifndef SSR_PERIOD_MS
#define SSR_PERIOD_MS 2000  // 2 second period for SSR time-proportional control
#endif

extern int16_t current_temperature_celsius;

#ifdef ESP32
Preferences pid_prefs;
#else
// EEPROM addresses for AVR
#define EEPROM_KP_ADDR 0
#define EEPROM_KI_ADDR 4
#define EEPROM_KD_ADDR 8
#define EEPROM_SETPOINT_ADDR 12
#define EEPROM_MAGIC_ADDR 14
#define EEPROM_MAGIC_VALUE 0xAB  // Magic byte to detect if EEPROM is initialized
#endif

int16_t pid_setpoint = 37;
uint8_t pid_holdback = 5;
bool pid_enabled = false;
uint8_t pid_current_power = 0; // 0% - 100%

// PID parameters
float pid_kp = 1;
float pid_ki = 1;
float pid_kd = 1;

// PID internal state
float pid_integral = 0.0;
float pid_last_error = 0.0;
unsigned long pid_last_time = 0;

// Software time-proportional control state
unsigned long ssr_cycle_start = 0;
bool ssr_state = false;

void pid_save_settings() {
#ifdef ESP32
  pid_prefs.begin("pid", false);
  pid_prefs.putFloat("kp", pid_kp);
  pid_prefs.putFloat("ki", pid_ki);
  pid_prefs.putFloat("kd", pid_kd);
  pid_prefs.putShort("setpoint", pid_setpoint);
  pid_prefs.end();
#else
  // AVR EEPROM implementation
  EEPROM.put(EEPROM_KP_ADDR, pid_kp);
  EEPROM.put(EEPROM_KI_ADDR, pid_ki);
  EEPROM.put(EEPROM_KD_ADDR, pid_kd);
  EEPROM.put(EEPROM_SETPOINT_ADDR, pid_setpoint);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
#endif
}

void pid_load_settings() {
#ifdef ESP32
  pid_prefs.begin("pid", true); // read-only mode
  pid_kp = pid_prefs.getFloat("kp", 1.0);
  pid_ki = pid_prefs.getFloat("ki", 2.0);
  pid_kd = pid_prefs.getFloat("kd", 1.0);
  pid_setpoint = pid_prefs.getShort("setpoint", 37);
  pid_prefs.end();
#else
  // AVR EEPROM implementation - check if initialized
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC_VALUE) {
    EEPROM.get(EEPROM_KP_ADDR, pid_kp);
    EEPROM.get(EEPROM_KI_ADDR, pid_ki);
    EEPROM.get(EEPROM_KD_ADDR, pid_kd);
    EEPROM.get(EEPROM_SETPOINT_ADDR, pid_setpoint);
  } else {
    // EEPROM not initialized, use defaults
    pid_kp = 1.0;
    pid_ki = 2.0;
    pid_kd = 1.0;
    pid_setpoint = 37;
  }
#endif
}

void pid_init() {
  // Load saved settings from flash
  pid_load_settings();
  
  pinMode(PID_OUTPUT_PIN, OUTPUT);
  pinMode(HEATER_LED_PIN, OUTPUT);
  digitalWrite(PID_OUTPUT_PIN, LOW);
  digitalWrite(HEATER_LED_PIN, LOW);
  
  pid_integral = 0.0;
  pid_last_error = 0.0;
  pid_last_time = millis();
  ssr_cycle_start = millis();
  ssr_state = false;
}

void pid_update_ssr() {
  // Software time-proportional control for SSR
  unsigned long now = millis();
  unsigned long elapsed = now - ssr_cycle_start;
  
  if (elapsed >= SSR_PERIOD_MS) {
    // Start new cycle
    ssr_cycle_start = now;
    elapsed = 0;
  }
  
  // Calculate on-time for this cycle
  unsigned long on_time = (SSR_PERIOD_MS * pid_current_power) / 100;
  
  // Set SSR state based on elapsed time
  bool new_state = (elapsed < on_time);
  
  if (new_state != ssr_state) {
    ssr_state = new_state;
    digitalWrite(PID_OUTPUT_PIN, ssr_state ? HIGH : LOW);
    digitalWrite(HEATER_LED_PIN, ssr_state ? HIGH : LOW);
  }
}

void pid_compute() {
  if (!pid_enabled) {
    pid_current_power = 0;
    pid_integral = 0.0;
    pid_last_error = 0.0;
    return;
  }
  
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0; // Convert to seconds
  
  if (dt < 1.0) {
    return; // Update at most every 1 second
  }
  
  pid_last_time = now;
  
  // Calculate error
  float error = pid_setpoint - current_temperature_celsius;
  
  // PID calculations
  pid_integral += error * dt;
  
  // Anti-windup: limit integral term
  float max_integral = 100.0 / pid_ki;
  if (pid_integral > max_integral) pid_integral = max_integral;
  if (pid_integral < -max_integral) pid_integral = -max_integral;
  
  float derivative = (error - pid_last_error) / dt;
  pid_last_error = error;
  
  // Calculate output
  float output = (pid_kp * error) + (pid_ki * pid_integral) + (pid_kd * derivative);
  
  // Convert PID output to power percentage (0-100%)
  // Clamp negative outputs to 0
  if (output < 0) {
    output = 0;
  }
  
  // Scale to 0-100% based on maximum expected PID output
  // Assuming max temp error of 400°C, with typical Kp=10, Ki=61, Kd=9:
  // Max P term: 10 * 400 = 4000
  // Max I term: limited by anti-windup to 100/Ki ≈ 1.64 at 61 Ki, so ~100
  // Max D term: reasonable max would be ~100 for aggressive tuning
  // Conservative max output estimate: ~500 for normal operation
  float max_output = 200.0;
  float power_percentage = (output / max_output) * 100.0;
  
  // Clamp to 0-100%
  if (power_percentage > 100.0) power_percentage = 100.0;
  
  pid_current_power = (uint8_t)power_percentage;
}
