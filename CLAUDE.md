# Project Instructions for Claude Code

## Project Overview

ESP32 MultiWii brushed drone flight controller with ESP-NOW wireless communication.

- **Board:** ESP32 MH ET Live MiniKit
- **Framework:** Arduino (ESP32 Core 3.0.7+)
- **License:** GPL (inherited from MultiWii)

## Key Files

| File | Purpose |
|------|---------|
| [config.h](config.h) | Main configuration (pins, PID, features) |
| [Output_ESP32.cpp](Output_ESP32.cpp) | Motor PWM control |
| [ESP_NOW_RX.cpp](ESP_NOW_RX.cpp) | Wireless receiver |
| [Sensors.cpp](Sensors.cpp) | IMU and battery monitoring |
| [ESP32_README.md](ESP32_README.md) | Hardware setup guide |

## Code Style

- Use `#if defined(ESP32)` guards for ESP32-specific code
- Keep AVR compatibility where practical
- Use ESP32 Core 3.x API: `ledcAttach(pin, freq, res)` and `ledcWrite(pin, duty)`
- Document pin assignments in both code and README

## Pin Safety Rules

**Never use these pins for outputs:**
- GPIO 0, 2, 12 - Strapping pins (can cause boot failures)
- GPIO 34-39 - Input-only pins

**Current safe assignments:**
- Motors: 13, 25, 14, 27
- Buzzer: 32
- LED: 33
- Battery ADC: 34

## Testing

Before committing changes:
1. Verify compilation with Arduino CLI or IDE
2. Check no strapping pins are used incorrectly
3. Ensure platform guards wrap AVR-specific code

## Hardware Circuits

### Buzzer (GPIO 32)
Uses 2N2222 NPN transistor driver:
- Base: 1kΩ resistor to GPIO 32
- Collector: Buzzer negative
- Emitter: GND
- Buzzer positive: 3.3V or 5V
- Optional: 1N4148 flyback diode for magnetic buzzers

See [README.md](README.md) for full schematic.

## Related Documentation

- [ESP32_README.md](ESP32_README.md) - Complete hardware/software setup
- [DEVELOPMENT_LOG.md](DEVELOPMENT_LOG.md) - Session history and decisions

## Compatible Transmitters

Located in `../New_FPV_TRANSMITTER_Code-MultiWii_ESP32-ESPNOW/`:

| Transmitter | Use Case |
|-------------|----------|
| `NEW_FPV_Transmitter_Code_ESP32_ESPNOW/` | Standard spring-return joysticks |
| `Transmitter_ESP32_No_Spring_Joy/` | No-spring throttle (linear mapping, input smoothing) |

**Data Structure Alignment:** Both transmitters and this receiver must use identical `__attribute__((packed))` structs:
- `struct_message` (7 bytes): throttle, yaw, pitch, roll, AUX1, AUX2, switches
- `struct_ack` (11 bytes): vbat, rssi, heading, pitch, roll, alt, flags

**RC Channel Mapping:**
- AUX1/AUX2: Toggle switches (0/1 → 1000/2000)
- AUX3: Left joystick button (switches bit0)
- AUX4: Right joystick button (switches bit1) - can trigger BOXBEEPERON

When modifying data structures, update ALL THREE locations:
1. [ESP_NOW_RX.cpp:17-38](ESP_NOW_RX.cpp#L17-L38) (this project)
2. `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino:76-95`
3. `Transmitter_ESP32_No_Spring_Joy.ino:57-75`
