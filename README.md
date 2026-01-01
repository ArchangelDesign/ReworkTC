# ReworkTC - PID Temperature Controller

A versatile PID temperature controller designed for soldering rework stations, reflow ovens, and other precision temperature control applications. Supports ESP32 (with display and Bluetooth) and Arduino Nano (minimal configuration). Features thermocouple temperature sensing, optional OLED display, and Bluetooth/Serial control interface.

## Features

- **PID Temperature Control**: Precise temperature regulation with tunable PID parameters (Kp, Ki, Kd)
- **Auto-Tune**: Automatic PID parameter optimization using relay-feedback method
- **MAX6675 Thermocouple Interface**: Accurate temperature readings up to 1024°C
- **Multiple Board Support**: ESP32 (full features), Arduino Uno, or Arduino Nano
- **OLED Display**: Real-time temperature, power percentage, and status display (ESP32)
- **Dual Control Interface**: Bluetooth and Serial command support
- **Time-Proportional SSR Control**: Smooth 2-second cycle PWM for solid-state relays
- **Persistent Settings**: PID parameters and setpoint stored in flash/EEPROM
- **Improved PID Algorithm**: Fast 250ms updates with derivative filtering and smart anti-windup
- **Open Source**: MIT licensed for easy modification and integration

## Hardware Requirements

### Supported Boards

#### ESP32 (Full Features)
- **Board**: Heltec WiFi Kit 32 or compatible ESP32
- **Features**: Display, Bluetooth, persistent settings, all commands
- **Environment**: `heltec_wifi_kit_32`

#### Arduino Uno (Recommended for Production)
- **Board**: Arduino Uno (ATmega328P)
- **Features**: Serial control, persistent EEPROM settings, reliable USB (ATmega16U2/FTDI)
- **Environment**: `uno`
- **Note**: More reliable serial communication than Nano clones with CH340

#### Arduino Nano (Budget Option)
- **Board**: Arduino Nano (AT, **Arduino Uno**, or **Arduino Nano**
- **MAX6675 Thermocouple Interface Module**
- **K-Type Thermocouple**
- **Solid State Relay (SSR)** for heater control
- **SSD1306 OLED Display** (128x64, I2C) - ESP32 only
- **Optional**: External FT232 USB-Serial module for reliable Nano serial communication
### Core Components
- **ESP32 Development Board** or **Arduino Nano**
- **MAX6675 Thermocouple Interface Module**
- **K-Type Thermocouple**
- **Solid State Relay (SSR)** for heater control
- **SSD1306 OLED Display** (128x64, I2C) - ESP32 only

### Connections

#### ESP32 Configuration

##### MAX6675 Thermocouple Module
- `CLK` → GPIO 18
- `CS` → GPIO 5
- `DO` → GPIO 19
- `VCC` → 3.3V
- `GND` → GND

##### SSR Control
- `SSR Signal` → GPIO 25
- `Status LED` → GPIO 22

##### OLED Displa/Uno Configuration

##### MAX6675 Thermocouple Module
- `CLK` → Digital Pin 7
- `CS` → Digital Pin 8
- `DO` → Digital Pin 6
- `VCC` → 5V
- `GND` → GND

##### SSR Control
- `SSR Signal` → Digital Pin 3 (PWM)
- `Status LED` → Digital Pin 13 (Built-in LED)

**Note**: For Nano with CH340 USB chip issues, connect external FT232 module to TX/RX pins for reliable serial communication.
- `DO` → Digital Pin 6
- `VCC` → 5V
- `GND` → GND

##### SSR Control
- `SSR Signal` → Digital Pin 3 (PWM)
- `Status LED` → Digital Pin 13 (Built-in LED)

## Software Requirements

- [PlatformIO](https://platformio.org/) (recommended) or Arduino IDE
- Required Libraries (ESP32 only):
  - Adafruit GFX Library
  - Adafruit SSD1306

## Building the Project

### Using PlatformIO (Recommended)

1. **Clone or download the repository**
   ```bash
   git clone <repository-url>
   cd ReworkTc
   ```

2. **Build for ESP32**
   ```bash
   pio run -e heltec_wiUno**
   ```bash
   pio run -e uno
   ```

4. **Build for Arduino Nano**
   ```bash
   pio run -e nanoatmega328
   ```

5  pio run -e nESP32**
   ```bash
   pio run -e heltec_wifi_kit_32 --target upload --upload-port COMX
   ```

6. **Upload to Arduino Uno**
   ```bash
   pio run -e uno --target upload --upload-port COMX
   ```

7. **Upload to Arduino Nano**
   ```bash
   pio run -e nanoatmega328 --target upload --upload-port COMX
   ```
   Replace `COMX` with your board's serial port (e.g., `COM7` on Windows, `/dev/ttyUSB0` on Linux)

8  ```

5. **Upload to Arduino Nano**
   ```bash
   pio run -e nanoatmega328 --target upload --upload-port COMX
   ```
   Replace `COMX` with your board's serial port (e.g., `COM7` on Windows, `/dev/ttyUSB0` on Linux)

6. **Monitor Serial Output**
   ```bash
   pio device monitor
   ```

### Using Arduino IDE

1. Install ESP32 board support
2. Install required libraries via Library Manager
3. Open `src/main.cpp` and compile
4. Upload to your ESP32 board

## Configuration

### Build Flags

Edit `platformio.ini` to customize pin assignments and features:

```ini
build_flags = 
    -DBT_ENABLED=1              ; Enable (1) or disable (0) Bluetooth
    -DDISABLE_DISPLAY=0         ; Disable display if not used
    -DPID_OUTPUT_PIN=25         ; SSR control pin
    -DHEATER_LED_PIN=22         ; Status LED pin
    -DSSR_PERIOD_MS=2000        ; SSR PWM cycle time (ms)
    -DMAX6675_CLK=18            ; MAX6675 clock pin(ESP32) and Serial. 

**Serial Settings:**
- ESP32: 115200 baud
- Arduino Uno/Nano: 9600 baud
- Format: 8-N-1 (8 data bits, no parity, 1 stop bit)

Commands are case-sensitive and end with newline (`\n`).

#### Available Commands

| Command | Description | Example |
|---------|-------------|---------|
| `SET:<temp>` | Set target temperature (0-400°C) | `SET:250` |
| `ON` | Enable PID controller | `ON` |
| `OFF` | Disable PID controller | `OFF` |
| `STATUS` | Get current status | `STATUS` |
| `KP:<value>` | Set proportional gain | `KP:10.0` |
| `KI:<value>` | Set integral gain | `KI:0.5` |
| `KD20
OK Setpoint=120 (saved)

TUNE
OK Auto-tune started. Wait 5-10 minutes...
WARNING: Monitor temperature! Stop if unstable.

[Wait for auto-tune to complete...]

Auto-tune complete! Kp=12.34 Ki=0.67 Kd=45.21

ON
OK PID=ON

STATUS
SETPOINT:120 ENABLED:1 TEMP:119.8 POWER:28
```

#### Automatic Tuning (Recommended)

The controller includes an auto-tune feature that uses relay-feedback method with Ziegler-Nichols tuning:

1. **Set target temperature**: `SET:120`
2. **Start auto-tune**: `TUNE`
3. **Wait 5-10 minutes** while the system oscillates
4. **Parameters automatically calculated and saved**

The auto-tune will:
- Heat to setpoint then cycle on/off
- Measure oscillation amplitude and period
- Calculate optimal PID values
- Save to EEPROM/flash automatically

**⚠️ Monitor temperature during auto-tune!** Stop if unstable.

#### Manual Tuning

Default PID values work well for most systems:
- Smart anti-windup protection for integral term
- Power output clamped to 0-100%
- Time-proportional control for SSR longevity
- Thermocouple open-circuit detection
- Derivative filtering reduces noise-induced instability
- Auto-tune monitors oscillations to prevent runaway
To manually adjust:

1. Start with defaults: `KP:10.0`, `KI:0.5`, `KD:50.0`
2. **If overshoots**: Reduce Kp or increase Kd
3. **If slow response**: Increase Kp
4. **If doesn't reach setpoint**: Increase Ki
5. **If oscillates**: Reduce Kp and Ki, increase Kd

Values are automatically saved to persistent storage.

#### PID Algorithm Details

- **Update rate**: 250ms (4 Hz) for responsive control
- **Derivative filtering**: Low-pass filter reduces thermocouple noise
- **Smart anti-windup**: Integral only accumulates when output is 5-95%
- **Direct percentage output**: 0-100% power to heater|
| `KD:<value>` | Set deri (ESP32)**
- Verify `BT_ENABLED=1` in build flags
- Check device appears as "ReworkTC"
- Ensure no other device is connected

**PID oscillates or overshoots**
- Run auto-tune: `TUNE` command
- If manual tuning: reduce Kp, increase Kd
- Verify SSR cycle time is appropriate (default 2000ms)

**Serial port won't open on Arduino Nano**
- **CH340 USB chip issue**: Common with cheap Nano clones
- **Solution 1**: Use Arduino Uno (has better ATmega16U2 USB chip)
- **Solution 2**: Connect external FT232 module to TX/RX pins
- **Solution 3**: Try different Nano board (CH340 quality varies)
- Note: This is a hardware limitation, not a firmware issue

**C# application can't connect to Nano**
- CH340 driver incompatibility with .NET SerialPort class
- Use Arduino Uno or external FT232 USB-Serial converter
- PlatformIO monitor and Python work fine, but some .NET implementations fail

ON
OK PID=ON

STATUS
SETPOINT:180 ENABLED:1 TEMP:175.5 POWER:45
```

**Via Bluetooth:**
- Connect to device named `ReworkTC`
- Send same commands as Serial

### PID Tuning

Default PID values are conservative. Adjust based on your system:

1. Start with: `KP:1.0`, `KI:2.0`, `KD:1.0`
2. Increase Kp for faster response
3. Increase Ki to eliminate steady-state error
4. Increase Kd to reduce overshoot

Values are automatically saved to flash memory.

## Display Information

The OLED displays:
- **Large**: Current temperature in °C
- **Power**: Heater power percentage (0-100%)
- **HEAT**: ON/OFF status
- **Set**: Target temperature (when PID enabled)

## Safety Features

- Temperature range limited to 0-400°C
- Anti-windup protection for integral term
- Power output clamped to 0-100%
- Time-proportional control for SSR longevity
- Thermocouple open-circuit detection

## Troubleshooting

**Display shows "Error: TC Open"**
- Check thermocouple connections
- Verify MAX6675 wiring
- Ensure thermocouple polarity is correct

**Temperature readings incorrect**
- Verify GPIO pin assignments match hardware
- Check thermocouple type (K-type required)
- Ensure proper grounding

**No Bluetooth connection**
- Verify `BT_ENABLED=1` in build flags
- Check device appears as "ReworkTC"
- Ensure no other device is connected

**PID oscillates or overshoots**
- Reduce Kp gain
- Increase Kd gain
- Reduce SSR cycle time if too slow

## License

Copyright (c) 2025 Black Horse Repairs LLC

Licensed under the MIT License. See project files for full license text.

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## Support

For support and questions, please open an issue on the repository.
