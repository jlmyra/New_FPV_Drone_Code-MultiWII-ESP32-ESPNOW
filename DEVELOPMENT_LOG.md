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

## Known Limitations

1. **Battery:** Requires voltage divider for 1S LiPo (4.2V max)
2. **ESP32 Core:** Requires version 3.x (tested with 3.0.7)
3. **Motors:** Brushed only (32kHz PWM)

## Future Considerations

- ADC calibration routine for battery accuracy
- ESP32-S3 support
- OTA updates via ESP-NOW
- Blackbox logging to SPIFFS
