/**
 * @file touch_interface.h
 * @brief Touch display ST7796S interface for ReworkTC
 * 
 * @copyright Copyright (c) 2026 Black Horse Repairs LLC
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
#include <Wire.h>
#include "I2C_Anything.h"
#include "pid_controller.h"

extern int16_t offset_temperature;

#define I2C_DEV_ADDR 0x55

String receivedMessage = "";

void send_i2c_Status()
{
    Wire.beginTransmission(I2C_DEV_ADDR);
    I2C_writeAnything(current_temperature_celsius);
    I2C_writeAnything(offset_temperature);
    I2C_writeAnything(pid_setpoint);
    I2C_writeAnything(pid_enabled);
    I2C_writeAnything(pid_current_power);
    I2C_writeAnything(pid_kp);
    I2C_writeAnything(pid_ki);
    I2C_writeAnything(pid_kd);
    I2C_writeAnything(pid_autotune_active);
    Wire.endTransmission();    // stop transmitting
}

void check_i2c_updates()
{
    char c;
    char message[8];
    memset(message, 0, sizeof(message)); // clean
    Wire.requestFrom(I2C_DEV_ADDR, 8);
    I2C_readAnything(message);
    String ValueToSave;
    String _first = (String)message[0];
    switch (_first.charAt(0))
    {
    case '0':
        // Nothing to do
        break;
    case 'H': // Start heating
        pid_enabled = true;
        break;
    case 'S': // Stop heating
        pid_enabled = false;
        break;
    case 'P': // Set P of PID
        ValueToSave = String(message);
        ValueToSave = ValueToSave.substring(ValueToSave.length() - 6);
        pid_kp = ValueToSave.toFloat();
        pid_save_settings();
        pid_load_settings();
        break;
    case 'I': // Set I of PID
        ValueToSave = String(message);
        ValueToSave = ValueToSave.substring(ValueToSave.length() - 6);
        pid_ki = ValueToSave.toFloat();
        pid_save_settings();
        pid_load_settings();
        break;
    case 'D': // Set D of PID
        ValueToSave = String(message);
        ValueToSave = ValueToSave.substring(ValueToSave.length() - 6);
        pid_kd = ValueToSave.toFloat();
        pid_save_settings();
        pid_load_settings();
        break;
    case 'T': // Setpoint
        ValueToSave = String(message);
        ValueToSave = ValueToSave.substring(ValueToSave.length() - 6);
        pid_setpoint = ValueToSave.toInt();
        pid_save_settings();
        pid_load_settings();
        break;
    default:
        break;
    case 'O': // Set offset
        ValueToSave = String(message);
        ValueToSave = ValueToSave.substring(ValueToSave.length() - 6);
        Serial.println(ValueToSave.toFloat());
        offset_temperature = ValueToSave.toInt();
        pid_save_settings();
        pid_load_settings();
        break;
    case 'A': // Start autotune
        if (pid_setpoint > 30 && pid_setpoint < 400) {
            //extern bool pid_autotune_active;
            pid_autotune_active = true;
            pid_enabled = true;
        }
    }
}