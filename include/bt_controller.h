/**
 * @file bt_controller.h
 * @brief Bluetooth command interface for ReworkTC
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

#if BT_ENABLED==1
#include <BluetoothSerial.h>
#endif

extern int16_t pid_setpoint;
extern uint8_t pid_holdback;
extern bool pid_enabled;
extern uint8_t pid_current_power;
extern float pid_kp;
extern float pid_ki;
extern float pid_kd;
extern int16_t current_temperature_celsius;
extern bool pid_autotune_active;
extern float pid_kp;
extern float pid_ki;
extern float pid_kd;
extern bool hold_power;
extern uint8_t max_power_limit;

extern void pid_save_settings();

#if BT_ENABLED==1
BluetoothSerial* SerialBT = nullptr;

void bt_init() {
  if (SerialBT == nullptr) {
    SerialBT = new BluetoothSerial();
  }
  if (SerialBT->begin("ReworkTC")) {
    Serial.println(F("Bluetooth initialized as 'ReworkTC'"));
  } else {
    Serial.println(F("Bluetooth init failed!"));
  }
}
#else
void bt_init() {
  Serial.println(F("Bluetooth disabled by build flag"));
}
#endif

void bt_process_commands() {
  // Check if there's a command from either Bluetooth or Serial
  Stream* input = nullptr;
  Stream* output = nullptr;
  bool from_bt = false;
  
#if BT_ENABLED==1
  if (SerialBT != nullptr && SerialBT->available()) {
    input = SerialBT;
    output = SerialBT;
    from_bt = true;
  } else
#endif
  if (Serial.available()) {
    input = &Serial;
    output = &Serial;
    from_bt = false;
  }
  
  if (input == nullptr) {
    return;
  }
  
  // Replace String with char buffer to save RAM
  char command[32];  // Fixed buffer instead of String object
  int idx = 0;
  
  // Read command into buffer
  while (input->available() && idx < 31) {
    char c = input->read();
    if (c == '\n' || c == '\r') break;
    command[idx++] = c;
  }
  command[idx] = '\0';
  
  // Trim leading/trailing spaces
  while (idx > 0 && (command[idx-1] == ' ' || command[idx-1] == '\t')) {
    command[--idx] = '\0';
  }
  
  if (strncmp(command, "SET:", 4) == 0) {
    // Command: SET:150 (set setpoint to 150°C)
    int value = atoi(command + 4);
    if (value >= 0 && value <= 400) {
      pid_setpoint = value;
      pid_save_settings();
      output->print(F("OK Setpoint="));
      output->print(pid_setpoint);
      output->println(F(" (saved)"));
#if BT_ENABLED==1
      if (from_bt) {
        Serial.print(F("BT: Setpoint set to "));
        Serial.print(pid_setpoint);
        Serial.println(F("°C"));
      }
#endif
    } else {
      output->println(F("ERROR Invalid setpoint range (0-400)"));
    }
  }
  else if (strcmp(command, "ON") == 0) {
    // Command: ON (enable PID)
    pid_enabled = true;
    output->println(F("OK PID=ON"));
#if BT_ENABLED==1
    if (from_bt) {
      Serial.println(F("BT: PID enabled"));
    }
#endif
  }
  else if (strcmp(command, "OFF") == 0) {
    // Command: OFF (disable PID)
    pid_enabled = false;
    output->println(F("OK PID=OFF"));
#if BT_ENABLED==1
    if (from_bt) {
      Serial.println(F("BT: PID disabled"));
    }
#endif
  }
  else if (strcmp(command, "STATUS") == 0) {
    // Command: STATUS (request current status)
    output->print(F("SETPOINT:"));
    output->print(pid_setpoint);
    output->print(F(" ENABLED:"));
    output->print(pid_enabled);
    output->print(F(" TEMP:"));
    output->print((float)current_temperature_celsius);
    output->print(F(" POWER:"));
    output->print(pid_current_power);
    output->print(F(" KP:"));
    output->print(pid_kp, 2);
    output->print(F(" KI:"));
    output->print(pid_ki, 2);
    output->print(F(" KD:"));
    output->print(pid_kd, 2);
    output->print(F(" HOLD:"));
    output->print(hold_power ? "1" : "0");
    output->print(F(" MAX_POWER:"));
    output->print(max_power_limit);
    output->print(F(" VERSION:"));
    output->print(REWORKTC_VERSION);
    output->print(F(" AT:"));
    output->println(pid_autotune_active ? F("ACTIVE") : F("INACTIVE"));
  }
  else if (strncmp(command, "KP:", 3) == 0) {
    // Command: KP:10.0 (set Kp parameter)
    float value = atof(command + 3);
    pid_kp = value;
    pid_save_settings();
    output->print(F("OK Kp="));
    output->print(pid_kp, 2);
    output->println(F(" (saved)"));
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print(F("BT: Kp set to "));
      Serial.println(pid_kp, 2);
    }
#endif
  }
  else if (strncmp(command, "KI:", 3) == 0) {
    // Command: KI:61.0 (set Ki parameter)
    float value = atof(command + 3);
    pid_ki = value;
    pid_save_settings();
    output->print(F("OK Ki="));
    output->print(pid_ki, 2);
    output->println(F(" (saved)"));
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print(F("BT: Ki set to "));
      Serial.println(pid_ki, 2);
    }
#endif
  }
  else if (strncmp(command, "KD:", 3) == 0) {
    // Command: KD:9.0 (set Kd parameter)
    float value = atof(command + 3);
    pid_kd = value;
    pid_save_settings();
    output->print(F("OK Kd="));
    output->print(pid_kd, 2);
    output->println(F(" (saved)"));
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print(F("BT: Kd set to "));
      Serial.println(pid_kd, 2);
    }
#endif
  }
  else if (strcmp(command, "HELP") == 0) {
    output->println(F("Commands:"));
    output->println(F("  SET:<temp>      - Set setpoint (0-400°C) [auto-saves]"));
    output->println(F("  ON              - Enable PID"));
    output->println(F("  OFF             - Disable PID"));
    output->println(F("  STATUS          - Get current status"));
    output->println(F("  KP:<value>      - Set Kp parameter [auto-saves]"));
    output->println(F("  KI:<value>      - Set Ki parameter [auto-saves]"));
    output->println(F("  KD:<value>      - Set Kd parameter [auto-saves]"));
    output->println(F("  MAXPOWER:<val>  - Set max power limit 0-100% [auto-saves]"));
    output->println(F("  POWER:<val>     - Manual power override 0-100%"));
    output->println(F("  RELEASE         - Release manual power control"));
    output->println(F("  TUNE            - Start PID auto-tune (requires setpoint)"));
    output->println(F("  HELP            - Show this help"));
  }
  else if (strcmp(command, "TUNE") == 0) {
    // Start auto-tune - will be handled in PID loop
    if (pid_setpoint > 30 && pid_setpoint < 400) {
      extern bool pid_autotune_active;
      pid_autotune_active = true;
      pid_enabled = true;
      output->println(F("OK Auto-tune started. Wait 5-10 minutes..."));
      output->println(F("WARNING: Monitor temperature! Stop if unstable."));
#if BT_ENABLED==1
      if (from_bt) {
        Serial.println(F("BT: Auto-tune started"));
      }
#endif
    } else {
      output->println(F("ERROR Set valid setpoint first (30-400°C)"));
    }
  }
  else if (strncmp(command, "POWER:", 6) == 0) {
    // Command: POWER:<value> (manual power control, for testing)
    int value = atoi(command + 6);
    if (value >= 0 && value <= 100) {
      hold_power = true;
      pid_current_power = (uint8_t)value;
      output->print(F("OK Power set to "));
      output->print(pid_current_power);
      output->println(F("% (manual override)"));
    } else {
      output->println(F("ERROR Invalid power range (0-100)"));
    }
  }
  else if (strcmp(command, "RELEASE") == 0) {
    // Command: POWER:RELEASE (release manual power control)
    hold_power = false;
    output->println(F("OK Power control released to PID"));
  }
  else if (strncmp(command, "MAXPOWER:", 9) == 0) {
    // Command: MAXPOWER:<value> (set maximum power limit for PID)
    int value = atoi(command + 9);
    if (value < 0 || value > 100) {
      output->println(F("ERROR Invalid max power range (0-100)"));
      return;
    } 
      max_power_limit = (uint8_t)value;
      pid_save_settings();
      output->print(F("OK Max power limit set to "));
      output->print(max_power_limit);
      output->println(F("% (saved)"));
  } else {
    output->println(F("ERROR Unknown command. Send HELP for commands."));
  }
}

void bt_send_status(float temperature, uint8_t power, bool heater_on) {
#if BT_ENABLED==1
  if (SerialBT != nullptr) {
    SerialBT->print(F("TEMP:"));
    SerialBT->print(temperature, 1);
    SerialBT->print(F(" POWER:"));
    SerialBT->print(power);
    SerialBT->print(F(" HEAT:"));
    SerialBT->print(heater_on ? F("ON") : F("OFF"));
    SerialBT->print(F(" SP:"));
    SerialBT->print(pid_setpoint);
    SerialBT->print(F(" EN:"));
    SerialBT->println(pid_enabled);
  }
#endif
}
