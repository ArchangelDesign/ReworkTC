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

#ifndef PID_MAX_OUTPUT
#define PID_MAX_OUTPUT 180  // Maximum PID output value for scaling
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
float pid_kp = 10.0;
float pid_ki = 0.5;
float pid_kd = 50.0;

// PID internal state
float pid_integral = 0.0;
float pid_last_error = 0.0;
float pid_last_derivative = 0.0;  // For derivative filtering
unsigned long pid_last_time = 0;

// Auto-tune state
bool pid_autotune_active = false;
uint8_t pid_autotune_cycles = 0;
float pid_autotune_peak_high = 0;
float pid_autotune_peak_low = 999;
unsigned long pid_autotune_last_crossing = 0;
unsigned long pid_autotune_period_sum = 0;
float pid_autotune_amplitude_sum = 0;
bool pid_autotune_relay_state = false;
float pid_autotune_output_step = 70.0; // % power for auto-tune
unsigned long pid_autotune_stall_timer = 0;

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
  pid_kp = pid_prefs.getFloat("kp", 10);
  pid_ki = pid_prefs.getFloat("ki", 0.5);
  pid_kd = pid_prefs.getFloat("kd", 50.0);
  pid_setpoint = pid_prefs.getShort("setpoint", 25);
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
    pid_ki = 0.5;
    pid_kd = 50;
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
    pid_autotune_active = false;
    return;
  }
  
  unsigned long now = millis();
  float dt = (now - pid_last_time) / 1000.0; // Convert to seconds
  
  if (dt < 0.25) {
    return; // Update every 250ms for better response
  }
  
  pid_last_time = now;
  
  // Auto-tune mode: relay-based tuning
  if (pid_autotune_active) {
    float temp = current_temperature_celsius;
    
    // Initialize on first run
    if (pid_autotune_cycles == 0 && pid_autotune_last_crossing == 0) {
      pid_autotune_last_crossing = now;
      pid_autotune_stall_timer = now;
      pid_autotune_peak_high = temp;
      pid_autotune_peak_low = temp;
      pid_autotune_relay_state = (temp < pid_setpoint);
      Serial.print("Auto-tune: Starting relay test with ");
      Serial.print(pid_autotune_output_step, 0);
      Serial.println("% power");
      Serial.print("Target: ");
      Serial.print(pid_setpoint);
      Serial.print("C, Current: ");
      Serial.print(temp);
      Serial.println("C");
    }
    
    // Check if stuck heating without reaching setpoint
    if (pid_autotune_relay_state && pid_autotune_cycles == 0) {
      if (now - pid_autotune_stall_timer > 60000) { // 60 seconds
        if (temp < pid_setpoint - 10) {
          // Not making progress, increase power
          pid_autotune_output_step += 10.0;
          if (pid_autotune_output_step > 90.0) pid_autotune_output_step = 90.0;
          pid_autotune_stall_timer = now;
          Serial.print("Auto-tune: Increasing power to ");
          Serial.print(pid_autotune_output_step, 0);
          Serial.println("% (not reaching setpoint)");
        }
      }
    }
    
    // Relay control: switch at setpoint
    bool should_heat = (temp < pid_setpoint);
    
    // Detect crossing through setpoint
    if (should_heat != pid_autotune_relay_state) {
      pid_autotune_relay_state = should_heat;
      
      unsigned long period = now - pid_autotune_last_crossing;
      
      // Valid crossing (at least 10 seconds since last)
      if (period > 10000 && pid_autotune_cycles > 0) {
        float amplitude = (pid_autotune_peak_high - pid_autotune_peak_low) / 2.0;
        
        Serial.print("Auto-tune cycle ");
        Serial.print(pid_autotune_cycles);
        Serial.print(": Period=");
        Serial.print(period / 1000.0);
        Serial.print("s, Amplitude=");
        Serial.print(amplitude);
        Serial.print("C, Peaks=");
        Serial.print(pid_autotune_peak_low);
        Serial.print("-");
        Serial.println(pid_autotune_peak_high);
        
        // Accumulate measurements
        pid_autotune_period_sum += period;
        pid_autotune_amplitude_sum += amplitude;
        pid_autotune_cycles++;
        
        // Reset peak tracking
        pid_autotune_peak_high = temp;
        pid_autotune_peak_low = temp;
        
        // After 4 cycles, calculate PID params
        if (pid_autotune_cycles >= 5) {
          float avg_amplitude = pid_autotune_amplitude_sum / 4.0;
          float avg_period = (pid_autotune_period_sum / 4.0) / 1000.0; // Convert to seconds
          
          // Ziegler-Nichols relay method
          float ku = (4.0 * pid_autotune_output_step) / (3.14159 * avg_amplitude);
          float tu = avg_period;
          
          // PID tuning (conservative for safety)
          pid_kp = 0.45 * ku;
          pid_ki = 0.54 * ku / tu;
          pid_kd = 0.075 * ku * tu;
          
          // Sanity limits
          if (pid_kp > 100) pid_kp = 100;
          if (pid_ki > 10) pid_ki = 10;
          if (pid_kd > 200) pid_kd = 200;
          
          // Save and finish
          pid_save_settings();
          pid_autotune_active = false;
          pid_enabled = false;  // Stop the heater, autotune complete
          pid_autotune_cycles = 0;
          pid_autotune_period_sum = 0;
          pid_autotune_amplitude_sum = 0;
          pid_integral = 0;
          
          Serial.println("=======================================");
          Serial.println("Auto-tune COMPLETE!");
          Serial.print("Ku (ultimate gain) = ");
          Serial.println(ku, 2);
          Serial.print("Tu (ultimate period) = ");
          Serial.print(tu, 2);
          Serial.println("s");
          Serial.print("New PID values: Kp=");
          Serial.print(pid_kp, 2);
          Serial.print(" Ki=");
          Serial.print(pid_ki, 2);
          Serial.print(" Kd=");
          Serial.println(pid_kd, 2);
          Serial.println("Values saved to memory.");
          Serial.println("=======================================");
        }
      } else if (pid_autotune_cycles == 0) {
        // First crossing, start counting
        pid_autotune_cycles = 1;
        Serial.println("Auto-tune: First crossing detected, starting measurements...");
      }
      
      pid_autotune_last_crossing = now;
      pid_autotune_stall_timer = now;
    }
    
    // Track peaks
    if (temp > pid_autotune_peak_high) {
      pid_autotune_peak_high = temp;
    }
    if (temp < pid_autotune_peak_low) {
      pid_autotune_peak_low = temp;
    }
    
    // Set output based on relay state
    pid_current_power = pid_autotune_relay_state ? (uint8_t)pid_autotune_output_step : 0;
    
    return;
  }
  
  // Normal PID mode
  // Calculate error
  float error = pid_setpoint - current_temperature_celsius;
  
  // Proportional term
  float p_term = pid_kp * error;
  
  // Integral term with anti-windup
  // Only accumulate integral when not saturated
  if (pid_current_power > 5 && pid_current_power < 95) {
    pid_integral += error * dt;
  }
  
  // Limit integral to prevent windup
  float max_integral = 50.0;  // Allow integral to contribute up to 50%
  if (pid_integral > max_integral / pid_ki) pid_integral = max_integral / pid_ki;
  if (pid_integral < -max_integral / pid_ki) pid_integral = -max_integral / pid_ki;
  
  float i_term = pid_ki * pid_integral;
  
  // Derivative term with filtering to reduce noise
  float derivative = (error - pid_last_error) / dt;
  pid_last_derivative = 0.7 * pid_last_derivative + 0.3 * derivative; // Low-pass filter
  float d_term = pid_kd * pid_last_derivative;
  
  pid_last_error = error;
  
  // Calculate output
  float output = p_term + i_term + d_term;
  
  // Clamp to 0-100%
  if (output < 0) output = 0;
  if (output > 100) output = 100;
  
  pid_current_power = (uint8_t)output;
}
