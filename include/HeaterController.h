/**
 * @file HeaterController.h
 * @brief Heater controller interface for ReworkTC
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
#include <max6675.h>
#include <ArduPID.h>

#define SSR_PERIOD_MS 2000

class HeaterController {
    public:
        HeaterController(String name, uint8_t outputPin, uint16_t ssrPeriodMs, uint8_t csPin);
        void begin();
        void compute();
        void setTargetTemperature(double target);
        void turnOn();
        void turnOff();
        void setMaxPower(uint8_t power);
        void setKp(double kp);
        void setKi(double ki);
        void setKd(double kd);
        double getCurrentPower();
        double getCurrentTemperature();
    private:
        uint8_t outputPin;
        uint16_t ssrPeriodMs;
        bool ssrState;
        uint8_t csPin;
        double targetTemperature;
        double currentTemperature;
        bool isOn;
        uint8_t maxPower;
        double currentPower;
        double k_p;
        double k_i;
        double k_d;
        String name;
        MAX6675* thermocouple;
        ArduPID* pid;
};

HeaterController::HeaterController(String name, uint8_t outputPin, uint16_t ssrPeriodMs, uint8_t csPin)
{
    this->name = name;
    this->outputPin = outputPin;
    this->ssrPeriodMs = ssrPeriodMs;
    this->csPin = csPin;
    this->targetTemperature = 0;
    this->isOn = false;
    this->maxPower = 100;
    this->currentPower = 0;
    this->k_p = 1.0f;
    this->k_i = 0.5f;
    this->k_d = 2.0f;
    this->ssrState = false;
}

void HeaterController::begin()
{
    pinMode(outputPin, OUTPUT);
    thermocouple = new MAX6675(MAX6675_CLK, csPin, MAX6675_DO);
    pid = new ArduPID();
    pid->begin(&currentTemperature, &currentPower, &targetTemperature, k_p, k_i, k_d);
    pid->setOutputLimits(0, 100);
    pid->setSampleTime(500);
    pid->start();    
}

double HeaterController::getCurrentTemperature()
{
    currentTemperature = thermocouple->readCelsius();
    return currentTemperature;
}

void HeaterController::compute()
{
    pid->compute();
}

void HeaterController::setTargetTemperature(double target)
{
    targetTemperature = target;
}

void HeaterController::turnOn()
{
    isOn = true;
}

void HeaterController::turnOff()
{
    isOn = false;
}

void HeaterController::setMaxPower(uint8_t power)
{
    maxPower = power;
}

void HeaterController::setKp(double kp)
{
    k_p = kp;
}

void HeaterController::setKi(double ki)
{
    k_i = ki;
}

void HeaterController::setKd(double kd)
{
    k_d = kd;
}

double HeaterController::getCurrentPower()
{
    return currentPower;
}