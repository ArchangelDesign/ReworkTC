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
#include <BluetoothSerial.h>

extern int16_t pid_setpoint;
extern uint8_t pid_holdback;
extern bool pid_enabled;
extern uint8_t pid_current_power;
extern float pid_kp;
extern float pid_ki;
extern float pid_kd;
extern int16_t current_temperature_celsius;

extern void pid_save_settings();

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

void bt_process_commands() {
  // Check if there's a command from either Bluetooth or Serial
  Stream* input = nullptr;
  Stream* output = nullptr;
  bool from_bt = false;
  
  if (SerialBT != nullptr && SerialBT->available()) {
    input = SerialBT;
    output = SerialBT;
    from_bt = true;
  } else if (Serial.available()) {
    input = &Serial;
    output = &Serial;
    from_bt = false;
  }
  
  if (input == nullptr) {
    return;
  }
  
  String command = input->readStringUntil('\n');
  command.trim();
  
  const char* source = from_bt ? "BT" : "Serial";
  
  if (command.startsWith("SET:")) {
    // Command: SET:150 (set setpoint to 150°C)
    int value = command.substring(4).toInt();
    if (value >= 0 && value <= 400) {
      pid_setpoint = value;
      pid_save_settings();
      output->printf("OK Setpoint=%d (saved)\n", pid_setpoint);
      if (from_bt) {
        Serial.printf("BT: Setpoint set to %d°C\n", pid_setpoint);
      }
    } else {
      output->println("ERROR Invalid setpoint range (0-400)");
    }
  }
  else if (command == "ON") {
    // Command: ON (enable PID)
    pid_enabled = true;
    output->println("OK PID=ON");
    if (from_bt) {
      Serial.println("BT: PID enabled");
    }
  }
  else if (command == "OFF") {
    // Command: OFF (disable PID)
    pid_enabled = false;
    output->println("OK PID=OFF");
    if (from_bt) {
      Serial.println("BT: PID disabled");
    }
  }
  else if (command == "STATUS") {
    // Command: STATUS (request current status)
    output->printf("SETPOINT:%d ENABLED:%d TEMP:%.1f POWER:%d\n", 
                    pid_setpoint, pid_enabled, (float)current_temperature_celsius, pid_current_power);
  }
  else if (command.startsWith("KP:")) {
    // Command: KP:10.0 (set Kp parameter)
    float value = command.substring(3).toFloat();
    pid_kp = value;
    pid_save_settings();
    output->printf("OK Kp=%.2f (saved)\n", pid_kp);
    if (from_bt) {
      Serial.printf("BT: Kp set to %.2f\n", pid_kp);
    }
  }
  else if (command.startsWith("KI:")) {
    // Command: KI:61.0 (set Ki parameter)
    float value = command.substring(3).toFloat();
    pid_ki = value;
    pid_save_settings();
    output->printf("OK Ki=%.2f (saved)\n", pid_ki);
    if (from_bt) {
      Serial.printf("BT: Ki set to %.2f\n", pid_ki);
    }
  }
  else if (command.startsWith("KD:")) {
    // Command: KD:9.0 (set Kd parameter)
    float value = command.substring(3).toFloat();
    pid_kd = value;
    pid_save_settings();
    output->printf("OK Kd=%.2f (saved)\n", pid_kd);
    if (from_bt) {
      Serial.printf("BT: Kd set to %.2f\n", pid_kd);
    }
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
    output->println("  HELP         - Show this help");
  }
  else {
    output->println("ERROR Unknown command. Send HELP for commands.");
  }
  
}

void bt_send_status(float temperature, uint8_t power, bool heater_on) {
  if (SerialBT != nullptr) {
    SerialBT->printf("TEMP:%.1f POWER:%d HEAT:%s SP:%d EN:%d\n", 
                  temperature, power, heater_on ? "ON" : "OFF", 
                  pid_setpoint, pid_enabled);
  }
}
