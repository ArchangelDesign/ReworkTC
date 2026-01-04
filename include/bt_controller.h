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

extern void pid_save_settings();

#if BT_ENABLED==1
BluetoothSerial* SerialBT = nullptr;

void bt_init() {
  if (SerialBT == nullptr) {
    SerialBT = new BluetoothSerial();
  }
  if (SerialBT->begin("ReworkTC")) {
    Serial.println("Bluetooth initialized as 'ReworkTC'");
  } else {
    Serial.println("Bluetooth init failed!");
  }
}
#else
void bt_init() {
  Serial.println("Bluetooth disabled by build flag");
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
  
  String command = input->readStringUntil('\n');
  command.trim();
  
  if (command.startsWith("SET:")) {
    // Command: SET:150 (set setpoint to 150°C)
    int value = command.substring(4).toInt();
    if (value >= 0 && value <= 400) {
      pid_setpoint = value;
      pid_save_settings();
      output->print("OK Setpoint=");
      output->print(pid_setpoint);
      output->println(" (saved)");
#if BT_ENABLED==1
      if (from_bt) {
        Serial.print("BT: Setpoint set to ");
        Serial.print(pid_setpoint);
        Serial.println("°C");
      }
#endif
    } else {
      output->println("ERROR Invalid setpoint range (0-400)");
    }
  }
  else if (command == "ON") {
    // Command: ON (enable PID)
    pid_enabled = true;
    output->println("OK PID=ON");
#if BT_ENABLED==1
    if (from_bt) {
      Serial.println("BT: PID enabled");
    }
#endif
  }
  else if (command == "OFF") {
    // Command: OFF (disable PID)
    pid_enabled = false;
    output->println("OK PID=OFF");
#if BT_ENABLED==1
    if (from_bt) {
      Serial.println("BT: PID disabled");
    }
#endif
  }
  else if (command == "STATUS") {
    // Command: STATUS (request current status)
    output->print("SETPOINT:");
    output->print(pid_setpoint);
    output->print(" ENABLED:");
    output->print(pid_enabled);
    output->print(" TEMP:");
    output->print((float)current_temperature_celsius);
    output->print(" POWER:");
    output->print(pid_current_power);
    output->print(" KP:");
    output->print(pid_kp, 2);
    output->print(" KI:");
    output->print(pid_ki, 2);
    output->print(" KD:");
    output->print(pid_kd, 2);
    output->print(" AT:");
    output->println(pid_autotune_active ? "ACTIVE" : "INACTIVE");
  }
  else if (command.startsWith("KP:")) {
    // Command: KP:10.0 (set Kp parameter)
    float value = command.substring(3).toFloat();
    pid_kp = value;
    pid_save_settings();
    output->print("OK Kp=");
    output->print(pid_kp, 2);
    output->println(" (saved)");
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print("BT: Kp set to ");
      Serial.println(pid_kp, 2);
    }
#endif
  }
  else if (command.startsWith("KI:")) {
    // Command: KI:61.0 (set Ki parameter)
    float value = command.substring(3).toFloat();
    pid_ki = value;
    pid_save_settings();
    output->print("OK Ki=");
    output->print(pid_ki, 2);
    output->println(" (saved)");
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print("BT: Ki set to ");
      Serial.println(pid_ki, 2);
    }
#endif
  }
  else if (command.startsWith("KD:")) {
    // Command: KD:9.0 (set Kd parameter)
    float value = command.substring(3).toFloat();
    pid_kd = value;
    pid_save_settings();
    output->print("OK Kd=");
    output->print(pid_kd, 2);
    output->println(" (saved)");
#if BT_ENABLED==1
    if (from_bt) {
      Serial.print("BT: Kd set to ");
      Serial.println(pid_kd, 2);
    }
#endif
  }
  else if (command == "HELP") {
    output->println("Commands:");
    output->println("  SET:<temp>   - Set setpoint (0-400°C) [auto-saves]");
    output->println("  ON           - Enable PID");
    output->println("  OFF          - Disable PID");
    output->println("  STATUS       - Get current status");
    output->println("  KP:<value>   - Set Kp parameter [auto-saves]");
    output->println("  KI:<value>   - Set Ki parameter [auto-saves]");
    output->println("  KD:<value>   - Set Kd parameter [auto-saves]");
    output->println("  TUNE         - Start PID auto-tune (requires setpoint)");
    output->println("  HELP         - Show this help");
  }
  else if (command == "TUNE") {
    // Start auto-tune - will be handled in PID loop
    if (pid_setpoint > 30 && pid_setpoint < 400) {
      extern bool pid_autotune_active;
      pid_autotune_active = true;
      pid_enabled = true;
      output->println("OK Auto-tune started. Wait 5-10 minutes...");
      output->println("WARNING: Monitor temperature! Stop if unstable.");
#if BT_ENABLED==1
      if (from_bt) {
        Serial.println("BT: Auto-tune started");
      }
#endif
    } else {
      output->println("ERROR Set valid setpoint first (30-400°C)");
    }
  }
  else {
    output->println("ERROR Unknown command. Send HELP for commands.");
  }
  
}

void bt_send_status(float temperature, uint8_t power, bool heater_on) {
#if BT_ENABLED==1
  if (SerialBT != nullptr) {
    SerialBT->print("TEMP:");
    SerialBT->print(temperature, 1);
    SerialBT->print(" POWER:");
    SerialBT->print(power);
    SerialBT->print(" HEAT:");
    SerialBT->print(heater_on ? "ON" : "OFF");
    SerialBT->print(" SP:");
    SerialBT->print(pid_setpoint);
    SerialBT->print(" EN:");
    SerialBT->println(pid_enabled);
  }
#endif
}
