# ReworkTC - PID Temperature Controller

A versatile PID temperature controller designed for soldering rework stations, reflow ovens, and other precision temperature control applications. Supports ESP32 (with display and Bluetooth) and Arduino Nano (minimal configuration). Features thermocouple temperature sensing, optional OLED display, and Bluetooth/Serial control interface.

## Features

- **PID Temperature Control**: Precise temperature regulation with tunable PID parameters (Kp, Ki, Kd)
- **MAX6675 Thermocouple Interface**: Accurate temperature readings up to 1024°C
- **Multiple Board Support**: ESP32 (full features) or Arduino Nano (minimal)
- **OLED Display**: Real-time temperature, power percentage, and status display (ESP32)
- **Dual Control Interface**: Bluetooth and Serial command support
- **Time-Proportional SSR Control**: Smooth 2-second cycle PWM for solid-state relays
- **Persistent Settings**: PID parameters and setpoint stored in flash memory (ESP32)
- **Open Source**: MIT licensed for easy modification and integration

## Hardware Requirements

### Supported Boards

#### ESP32 (Full Features)
- **Board**: Heltec WiFi Kit 32 or compatible ESP32
- **Features**: Display, Bluetooth, persistent settings, all commands
- **Environment**: `heltec_wifi_kit_32`

#### Arduino Nano (Minimal)
- **Board**: Arduino Nano (ATmega328P)
- **Features**: Serial control only, no display, no Bluetooth
- **Environment**: `nanoatmega328`
- **Note**: Lower memory footprint, ideal for embedded applications

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

##### OLED Display (I2C)
- `SDA` → GPIO 4 (default I2C)
- `SCL` → GPIO 15 (default I2C)
- `RST` → GPIO 16
- `VCC` → 3.3V
- `GND` → GND

#### Arduino Nano Configuration

##### MAX6675 Thermocouple Module
- `CLK` → Digital Pin 7
- `CS` → Digital Pin 8
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
   pio run -e heltec_wifi_kit_32
   ```

3. **Build for Arduino Nano**
   ```bash
   pio run -e nanoatmega328
   ```

4. **Upload to ESP32**
   ```bash
   pio run -e heltec_wifi_kit_32 --target upload --upload-port COMX
   ```

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
    -DMAX6675_CLK=18            ; MAX6675 clock pin
    -DMAX6675_CS=5              ; MAX6675 chip select pin
    -DMAX6675_DO=19             ; MAX6675 data output pin
```

### Upload Port

Update the upload port in `platformio.ini`:
```ini
upload_port = COM7              ; Windows
upload_port = /dev/ttyUSB0      ; Linux
```

## Usage

### Command Interface

The controller accepts commands via both Bluetooth and Serial (115200 baud). Commands are case-sensitive and end with newline (`\n`).

#### Available Commands

| Command | Description | Example |
|---------|-------------|---------|
| `SET:<temp>` | Set target temperature (0-400°C) | `SET:250` |
| `ON` | Enable PID controller | `ON` |
| `OFF` | Disable PID controller | `OFF` |
| `STATUS` | Get current status | `STATUS` |
| `KP:<value>` | Set proportional gain | `KP:10.0` |
| `KI:<value>` | Set integral gain | `KI:61.0` |
| `KD:<value>` | Set derivative gain | `KD:9.0` |
| `HELP` | Display command list | `HELP` |

#### Command Examples

**Via Serial Monitor:**
```
SET:180
OK Setpoint=180 (saved)

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
