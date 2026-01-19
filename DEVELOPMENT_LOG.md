# Development Log

This document records development sessions for the ESP32 MultiWii drone flight controller.

---

## Session: January 14-15, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Sonnet 4.5

### Summary

Configured and optimized the ESP32 MH ET Live MiniKit version of the MultiWii brushed drone flight controller, including pin assignments, critical bug fixes, and hardware interface setup.

### Changes Made

#### 1. Motor Pin Configuration

**Problem:** Original pins were inconsistent between config.h and Output_ESP32.cpp

**Solution:** Standardized on safe pins:
- Motor 1 (Rear Right): GPIO 13
- Motor 2 (Front Right): GPIO 25
- Motor 3 (Rear Left): GPIO 14
- Motor 4 (Front Left): GPIO 27

#### 2. GPIO 2 Strapping Pin Fix

**Problem:** LED on GPIO 2 could cause boot failures

**Solution:** Moved LED to GPIO 33

**Files:** config.h:516

#### 3. AVR Code Platform Guards

**Problem:** AVR-specific pinMode calls executing on ESP32

**Solution:** Added `#if !defined(ESP32)` guards

**Files:** MultiWii.cpp:637-643

#### 4. PWM API Update

**Problem:** Hardcoded PWM channels instead of pin numbers

**Solution:** Updated to ESP32 Core 3.x API using pin numbers

**Files:** def.h:562-563, Output_ESP32.cpp:13-15

#### 5. Battery Monitoring

**Problem:** 4.2V LiPo exceeds ESP32 ADC limit (3.6V)

**Solution:** Documented voltage divider requirement and added ADC configuration

**Files:** Sensors.cpp:1590-1598, ESP32_README.md

### Issues Resolved

| Issue | Severity | Solution |
|-------|----------|----------|
| GPIO 2 strapping pin for LED | HIGH | Moved to GPIO 33 |
| Motor pin inconsistency | HIGH | Standardized on 13,25,14,27 |
| AVR pinMode on ESP32 | MEDIUM | Added platform guards |
| PWM channel hardcoding | MEDIUM | Use pin numbers (Core 3.x) |
| Battery ADC overvoltage | HIGH | Voltage divider circuit |

### Build Verification

- **Status:** Success
- **Program Size:** 940,001 bytes (71%)
- **RAM Usage:** 45,512 bytes (13%)

---

## Transmitter Switch Configuration

### ESP-NOW Data Structure

The transmitter must send data matching this structure (defined in [ESP_NOW_RX.cpp:17-25](ESP_NOW_RX.cpp#L17-L25)):

```cpp
typedef struct struct_message {
  uint8_t throttle;  // 0-255 (mapped to 1000-2000)
  uint8_t yaw;       // 0-255 (mapped to 1000-2000)
  uint8_t pitch;     // 0-255 (mapped to 1000-2000)
  uint8_t roll;      // 0-255 (mapped to 1000-2000)
  uint8_t AUX1;      // 0 or 1 (switch state)
  uint8_t AUX2;      // 0 or 1 (switch state)
  uint8_t switches;  // Bit flags (unused currently)
} struct_message;
```

### AUX Channel Mapping

| Channel | RC Value | Purpose |
|---------|----------|---------|
| AUX1 | 1000/2000 | Primary flight mode switch |
| AUX2 | 1000/2000 | Secondary function switch |

**Note:** AUX1/AUX2 are binary (0 or 1) and mapped to 1000 or 2000 PWM values.

### Available Flight Modes (Box Functions)

Flight modes are defined in [types.h:33-79](types.h#L33-L79). Available modes depend on enabled features:

| Mode | Requires | Description |
|------|----------|-------------|
| BOXARM | Always | Arm/disarm motors |
| BOXANGLE | ACC | Self-level mode (limited angles) |
| BOXHORIZON | ACC | Self-level + acro at extremes |
| BOXBARO | BARO sensor | Altitude hold |
| BOXMAG | Always | Heading hold |
| BOXHEADFREE | HEADFREE defined | Headless mode |
| BOXBEEPERON | BUZZER defined | Buzzer on demand |

### Arming Configuration

Arming is configured in [config.h:234-238](config.h#L234-L238):

```cpp
#define ALLOW_ARM_DISARM_VIA_TX_YAW      // Throttle down + yaw right = arm
//#define ALLOW_ARM_DISARM_VIA_TX_ROLL   // Throttle down + roll right = arm
```

**Default:** Throttle minimum + yaw right to arm, yaw left to disarm.

**Safety:** `ONLYARMWHENFLAT` (config.h:232) prevents arming when tilted.

### Configuring AUX Switch Functions

Flight modes are assigned to AUX channels via the MultiWii GUI configurator or by modifying EEPROM values. The `rcOptions[]` array maps switch positions to enabled modes.

**Typical Setup:**
- AUX1 LOW (1000): Acro mode (no stabilization)
- AUX1 HIGH (2000): Angle mode (self-leveling)

### Failsafe Behavior

When signal is lost for >1 second ([ESP_NOW_RX.cpp:103-106](ESP_NOW_RX.cpp#L103-L106)):
- Throttle: 0
- Yaw/Pitch/Roll: 128 (center)
- AUX1/AUX2: 0 (low)
- switches: 0

### Transmitter Requirements

Your ESP32 transmitter must:
1. Match the `struct_message` layout exactly
2. Know the receiver's MAC address (printed at boot)
3. Send packets at regular intervals (10-50ms recommended)
4. Use values 0-255 for sticks, 0-1 for switches

---

## Session: January 16, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Added buzzer driver circuit documentation using 2N2222 NPN transistor.

### Changes Made

#### 1. Buzzer Circuit Documentation

**Problem:** ESP32 GPIO can only source ~12mA, insufficient for most buzzers

**Solution:** Documented 2N2222 transistor driver circuit with:
- 1kΩ base resistor
- Optional 1N4148 flyback diode for magnetic buzzers
- Schematic, component list, and pinout diagram

**Files:** README.md:31-79

### Hardware Documentation Added

| Component | Details |
|-----------|---------|
| Q1 | 2N2222 NPN transistor (TO-92) |
| R1 | 1kΩ base current limiter |
| D1 | 1N4148 flyback diode (optional) |
| Buzzer | 3.3V-5V passive recommended |

---

## Known Limitations

1. **Battery:** Requires voltage divider for 1S LiPo (4.2V max)
2. **ESP32 Core:** Requires version 3.x (tested with 3.0.7)
3. **Motors:** Brushed only (32kHz PWM)

---

## Session: January 16, 2026 (Part 2)

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Verified data packet alignment between flight controller receiver and both transmitter variants. Added `__attribute__((packed))` to transmitter code for guaranteed binary compatibility.

### Changes Made

#### 1. Data Packet Alignment Verification

**Analysis:** Compared `struct_message` and `struct_ack` between:
- `ESP_NOW_RX.cpp` (flight controller receiver)
- `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino` (spring-return joysticks)
- `Transmitter_ESP32_No_Spring_Joy.ino` (no-spring throttle)

**Result:** All structures aligned correctly:

| Structure | Size | Fields |
|-----------|------|--------|
| `struct_message` | 7 bytes | throttle, yaw, pitch, roll, AUX1, AUX2, switches |
| `struct_ack` | 11 bytes | vbat, rssi, heading, pitch, roll, alt, flags |

#### 2. Added `__attribute__((packed))` to Transmitter

**Problem:** Receiver used `__attribute__((packed))` but standard transmitter did not

**Solution:** Added `__attribute__((packed))` to both structs in `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino`

**Files Modified:**
- `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino:76` - `struct_message`
- `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino:87` - `struct_ack`

**Note:** `Transmitter_ESP32_No_Spring_Joy.ino` already had packed structs (no changes needed)

#### 3. Documentation Updates

Updated documentation to reflect transmitter variants and data structure requirements:
- `README.md` - Added both transmitter variants and telemetry structure
- `CLAUDE.md` - Added transmitter references and sync instructions
- `DEVELOPMENT_LOG.md` - This session entry

### Transmitter Variants Documented

| Variant | Features |
|---------|----------|
| `NEW_FPV_Transmitter_Code_ESP32_ESPNOW` | Spring-return joysticks, 10-bit calibration scale |
| `Transmitter_ESP32_No_Spring_Joy` | Linear throttle mapping, input smoothing (EMA), deadzone, 12-bit ADC |

---

## Session: January 16, 2026 (Part 3)

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Mapped joystick buttons to AUX3/AUX4 channels to enable additional flight controller functions like buzzer activation.

### Changes Made

#### 1. Joystick Button to AUX Channel Mapping

**Problem:** The `switches` byte in `struct_message` was transmitted but not mapped to any RC channel

**Solution:** Added mapping in `ESP_NOW_RX.cpp` to decode joystick buttons to AUX3/AUX4

**Code Added:**
```cpp
// Map joystick buttons from switches byte to AUX3/AUX4
// bit0 = left joystick button  -> AUX3
// bit1 = right joystick button -> AUX4 (can be used for BOXBEEPERON)
espnow_rcData[AUX3] = (MyData.switches & 0x01) ? 2000 : 1000;
espnow_rcData[AUX4] = (MyData.switches & 0x02) ? 2000 : 1000;
```

**Files Modified:**
- `ESP_NOW_RX.cpp:171-175`

### RC Channel Summary

| Channel | Source | Purpose |
|---------|--------|---------|
| THROTTLE | Left stick Y | Motor power |
| YAW | Left stick X | Rotation |
| PITCH | Right stick Y | Forward/back tilt |
| ROLL | Right stick X | Left/right tilt |
| AUX1 | Toggle switch 1 | Flight mode (e.g., ANGLE) |
| AUX2 | Toggle switch 2 | Secondary function |
| AUX3 | Left joystick button | Available for box functions |
| AUX4 | Right joystick button | BOXBEEPERON (buzzer) |

### Buzzer Activation

To activate the buzzer with the right joystick button:
1. Configure BOXBEEPERON to respond to AUX4 HIGH via MultiWii GUI or config
2. Press right joystick button → AUX4 goes HIGH (2000) → buzzer sounds

---

## Future Considerations

- ADC calibration routine for battery accuracy
- ESP32-S3 support
- OTA updates via ESP-NOW
- Blackbox logging to SPIFFS
