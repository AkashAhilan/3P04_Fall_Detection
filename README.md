# 3P04 Fall Detection System

A smart fall detection mat using dual load cells (Front side and Back) and an ESP32 microcontroller to detect and alert on potential falls.
**_Note .BAK FILES ARE BACKUP NOT COMPILED CODE_**

## Overview

This project monitors weight distribution across two pressure-sensitive load cells. When a sudden impact is detected, it enters a verification state that can be cancelled within 3 seconds using a button. If not cancelled, an alert is triggered and displayed on an LCD screen.

## Hardware Requirements

- **Microcontroller**: ESP32 (PlatformIO with pico32 environment)
- **Pressure Sensors**: 2× HX711 Load Cells (Analog-to-Digital Converters)
- **Display**: DFRobot RGB LCD1602 (16×2 characters via I2C)
- **Button**: Momentary pushbutton for alert cancellation
- **Power**: 5V supply for load cells and LCD

## Hardware Connections

### Load Cells (HX711)

- **LoadCell 1**:
  - DOUT → GPIO 34
  - SCK → GPIO 25
- **LoadCell 2**:
  - DOUT → GPIO 35
  - SCK → GPIO 26

### Display (I2C)

- **SDA** → GPIO 22
- **SCL** → GPIO 19
- **Address**: 0x6B

### Button

- **Cancel Button** → GPIO (UNKNOWN) (INPUT_PULLUP) SOON TO IMPLEMENT

## How It Works

### State Machine

1. **NORMAL**: Continuously monitors weight. Displays "Monitoring" on LCD.
2. **VERIFYING_FALL**: Impact detected. User has 3 seconds to press button to cancel.
3. **ALERT_TRIGGERED**: Fall confirmed. Displays alert on LCD until button is pressed.

### Detection Logic

- Combines readings from both load cells: `totalForce = abs(LoadCell_1) + abs(LoadCell_2)`
- Triggers alarm when `totalForce > IMPACT_THRESHOLD` (50.0 by default)
- Can be adjusted via `IMPACT_THRESHOLD` constant

## Software Setup

### Installation

1. Open project in PlatformIO
2. Build for `pico32` environment:
   ```bash
   pio run --environment pico32
   ```
3. Upload to device:
   ```bash
   pio run --target upload --environment pico32 --upload-port COM10
   ```

### Serial Monitor

- Connect at **115200 baud**
- View real-time load cell readings

## Serial Commands

- Send `t` over serial to manually calibrate (tare) the load cells

## Key Configuration

Edit these values in `main.cpp` to tune behavior:

```cpp
const float IMPACT_THRESHOLD = 50.0;          // Impact detection sensitivity
const unsigned long CANCEL_WINDOW = 3000;     // 3 seconds to cancel after impact
const unsigned long debounce = 200;           // Button debounce time (ms)
```

## Project Structure

```
├── src/
│   ├── main.cpp              # Main application
│   ├── WORKINGCODE.cpp.bak   # Backup of stable version
│   └── test1.cpp
├── lib/
├── include/
└── platformio.ini
```

## Dependencies

- `HX711_ADC` library (load cell communication)
- `DFRobot_RGBLCD1602` library (LCD display)
- Arduino framework for ESP32
