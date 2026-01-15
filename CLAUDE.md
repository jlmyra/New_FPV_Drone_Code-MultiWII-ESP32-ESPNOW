# Claude Code Session Documentation

This document records the AI-assisted development session for converting the MultiWii drone flight controller code to ESP32 with ESP-NOW wireless communication.

**Date:** January 14-15, 2026
**AI Assistant:** Claude Sonnet 4.5
**Developer:** jlmyra

---

## Session Overview

This session focused on configuring and optimizing the ESP32 MH ET Live MiniKit version of the MultiWii brushed drone flight controller, including pin assignments, critical bug fixes, and hardware interface setup.

---

## Work Completed

### 1. Motor Pin Configuration

**Objective:** Configure motor pins for ESP32 MH ET Live MiniKit that are safe and avoid boot issues.

**Original Pins (Inconsistent):**
- Motor 1: GPIO 25
- Motor 2: GPIO 26
- Motor 3: GPIO 27
- Motor 4: GPIO 16 (config.h) vs GPIO 14 (Output_ESP32.cpp) - **Conflict**

**New Pin Configuration:**
- Motor 1 (Rear Right): **GPIO 13** - PWM, CW rotation
- Motor 2 (Front Right): **GPIO 25** - PWM, CCW rotation
- Motor 3 (Rear Left): **GPIO 14** - PWM, CCW rotation
- Motor 4 (Front Left): **GPIO 27** - PWM, CW rotation

**Rationale:**
- Avoided GPIO 12 (strapping pin - can cause boot failures)
- All pins are safe for PWM output
- No conflicts with I2C, SPI, or other peripherals
- QUAD X configuration for optimal flight dynamics

**Files Modified:**
- `config.h` - Updated pin definitions (lines 507-510)
- `Output_ESP32.cpp` - Fixed comments to match actual pins (lines 26-29)
- `MultiWii_ESP32-ESPNOW.ino` - Updated header documentation (line 15)
- `ESP32_README.md` - Added motor layout diagram and pin table

---

### 2. Critical Safety Fixes

#### Issue A: GPIO 2 Strapping Pin (HIGH PRIORITY)

**Problem:** LED was on GPIO 2, a boot strapping pin that can cause boot failures if pulled LOW during reset.

**Solution:** Moved LED from GPIO 2 to GPIO 33 (safe pin)

**Files Modified:**
- `config.h:516` - Changed `ESP32_LED_PIN` from 2 to 33
- `MultiWii_ESP32-ESPNOW.ino:19` - Updated documentation
- `ESP32_README.md` - Updated pin tables

#### Issue B: AVR-Specific Code on ESP32

**Problem:** Arduino AVR pinMode calls (GPIO 3, 5, 6, 9) were executing on ESP32, setting invalid pins.

**Solution:** Added platform guards `#if !defined(ESP32)` around AVR-specific code

**Files Modified:**
- `MultiWii.cpp:637-643` - Wrapped pinMode calls in platform guard

#### Issue C: Inconsistent PWM Channel Definitions

**Problem:** Hardcoded PWM channel numbers instead of using pin numbers (ESP32 Core 3.x API)

**Solution:** Updated all `ledcWrite()` calls to use pin numbers consistently

**Files Modified:**
- `def.h:562-563` - Changed buzzer macros to use `ESP32_BUZZER_PIN`
- `Output_ESP32.cpp:13-15` - Removed unused channel definitions, added API notes

---

### 3. Battery Voltage Monitoring (1S LiPo)

**Objective:** Configure safe battery monitoring for 1S LiPo (3.0V - 4.2V range)

**Challenge:** 4.2V fully charged exceeds ESP32's 3.6V ADC limit (even with 11dB attenuation)

**Solution:** Documented two options with hardware recommendations

#### Option 1: 2:1 Voltage Divider (Recommended)

**Circuit:**
```
Battery+ ----[10kΩ]----+----[10kΩ]---- GND
                       |
                    GPIO34
```

**Result:**
- 4.2V → 2.1V at GPIO34 (safe!)
- 3.0V → 1.5V at GPIO34 (safe!)

#### Option 2: 11dB Attenuation Only (Risky)

**Warning:** Only safe up to 3.6V, full charge (4.2V) will damage ADC

**ADC Configuration Added:**
```cpp
analogReadResolution(12);        // 12-bit ADC (0-4095)
analogSetAttenuation(ADC_11db);  // 0-3.6V range
```

**Files Modified:**
- `Sensors.cpp:1590-1598` - Added ADC initialization in `initSensors()`
- `ESP32_README.md:79-131` - Added complete battery monitoring section with:
  - Circuit diagrams
  - Component lists
  - Calibration instructions
  - Voltage threshold recommendations

**Voltage Thresholds for 1S LiPo:**
- `VBATLEVEL_WARN1`: 37 (3.7V - low battery)
- `VBATLEVEL_WARN2`: 35 (3.5V - very low)
- `VBATLEVEL_CRIT`: 33 (3.3V - critical!)

---

### 4. ESP32 Arduino Core Version Documentation

**Required Version:** ESP32 Arduino Core **3.0.7** (or compatible 3.x)

**API Changes in Core 3.x:**
- Uses `ledcAttach(pin, frequency, resolution)` instead of `ledcSetup()`
- Uses `ledcWrite(pin, duty)` with pin numbers instead of channels
- Core 2.x users must modify PWM code in `Output_ESP32.cpp`

**Files Modified:**
- `ESP32_README.md:139-155` - Added version requirements and API notes

---

## Complete Pin Assignment Summary

| Function | GPIO | Type | Notes |
|----------|------|------|-------|
| Motor 1 (Rear Right) | 13 | PWM Output | CW rotation, 32kHz |
| Motor 2 (Front Right) | 25 | PWM Output | CCW rotation, 32kHz |
| Motor 3 (Rear Left) | 14 | PWM Output | CCW rotation, 32kHz |
| Motor 4 (Front Left) | 27 | PWM Output | CW rotation, 32kHz |
| Buzzer | 32 | PWM Output | 2kHz, 8-bit |
| LED | 33 | Digital Output | Safe pin |
| Battery Monitor | 34 | ADC Input | With voltage divider |
| MPU6050 SDA | 21 | I2C Data | 400kHz |
| MPU6050 SCL | 22 | I2C Clock | 400kHz |

**All pins verified safe:**
- No strapping pins used for critical functions
- No input-only pins (34-39) used for output
- No conflicts between peripherals
- Boot-safe configuration

---

## Code Quality Improvements

### Platform Compatibility
- Added `#if defined(ESP32)` guards throughout
- Separated AVR and ESP32-specific code
- Maintained backwards compatibility with original Arduino version

### Documentation
- Added comprehensive motor layout diagram (QUAD X)
- Documented all pin functions and safe usage
- Added calibration procedures
- Included circuit diagrams for voltage divider

### Safety
- Fixed strapping pin issues (GPIO 2)
- Protected ADC from overvoltage (voltage divider)
- Added battery monitoring thresholds
- Documented all safety considerations

---

## Compilation Verification

**Build Status:** ✅ Success

**Build Details:**
- **Platform:** ESP32 Arduino Core 3.0.7
- **Board:** ESP32 Dev Module
- **Program Size:** 940,001 bytes (71% of 1,310,720 bytes)
- **RAM Usage:** 45,512 bytes (13% of 327,680 bytes)
- **All features:** Motors, ESP-NOW, I2C, ADC, PWM - Verified

---

## Git Commit History

### Commit 1: `d38ae58` - Configure ESP32 motor pins and fix critical issues
**Changes:**
- Motor pin configuration (13, 25, 14, 27)
- Buzzer moved to GPIO 32
- LED moved from GPIO 2 to GPIO 33 (strapping pin fix)
- Platform guards for AVR code
- PWM channel consistency fixes
- Motor layout diagram added

**Files Changed:** 7 files, +77/-39 lines

### Commit 2: `88e7871` - Add 1S LiPo battery monitoring configuration
**Changes:**
- ADC configuration (12-bit, 11dB attenuation)
- Voltage divider documentation
- Calibration instructions
- Safety warnings

**Files Changed:** 2 files, +65 insertions

### Commit 3: `d004f50` - Document ESP32 Arduino Core version requirement
**Changes:**
- Added version requirement (3.0.7)
- Documented API differences
- Compatibility notes

**Files Changed:** 1 file, +9/-1 lines

---

## Issues Identified and Resolved

| Issue | Severity | Status | Solution |
|-------|----------|--------|----------|
| GPIO 2 strapping pin for LED | HIGH | ✅ Fixed | Moved to GPIO 33 |
| Motor pin inconsistency | HIGH | ✅ Fixed | Standardized on 13,25,14,27 |
| AVR pinMode on ESP32 | MEDIUM | ✅ Fixed | Added platform guards |
| PWM channel hardcoding | MEDIUM | ✅ Fixed | Use pin numbers (Core 3.x) |
| Battery ADC overvoltage risk | HIGH | ✅ Documented | Voltage divider circuit |
| No ADC configuration | MEDIUM | ✅ Fixed | Added 12-bit + 11dB config |

---

## Testing Recommendations

### Before First Flight:

1. **Pin Verification:**
   - Use multimeter to verify motor connections
   - Test each motor individually with low throttle
   - Verify rotation directions match diagram

2. **Battery Monitor:**
   - Build voltage divider circuit
   - Test with multimeter at various voltages
   - Calibrate VBATSCALE value
   - Verify low battery warnings trigger correctly

3. **System Integration:**
   - Test ESP-NOW communication
   - Verify IMU readings (MPU6050)
   - Test buzzer functionality
   - Verify LED status indicators

4. **Flight Testing:**
   - Remove propellers for initial tests
   - Test throttle response
   - Verify control inputs (roll, pitch, yaw)
   - Test failsafe behavior
   - Gradually increase throttle with propellers on

---

## Hardware Shopping List

### Battery Monitoring:
- 2x 10kΩ resistors (1% tolerance)
- Small perfboard or direct solder

### Optional Improvements:
- External LED for better visibility (already on GPIO 33)
- Active buzzer for GPIO 32 (2kHz capable)
- Battery connector with voltage divider pre-built

---

## Known Limitations

1. **Battery Voltage:**
   - Requires voltage divider for 4.2V max (1S LiPo full charge)
   - Without divider, max safe voltage is ~3.6V
   - Calibration needed for accurate readings

2. **ESP32 Core Version:**
   - Code requires Core 3.x (tested with 3.0.7)
   - Core 2.x requires PWM code modifications
   - Not compatible with ESP32-S2/S3/C3 without changes

3. **Motor Control:**
   - Brushed motors only (32kHz PWM)
   - Not suitable for brushless ESCs
   - No oneshot/multishot support

---

## Future Improvements to Consider

1. **Add ADC calibration routine** in setup for better battery accuracy
2. **Create voltage divider PCB** for easier assembly
3. **Add ESP32-S3 support** for future hardware upgrades
4. **Implement OTA updates** via ESP-NOW or WiFi
5. **Add blackbox logging** to SPIFFS or SD card
6. **Create GUI configurator** for ESP-NOW control parameters

---

## Resources

### Documentation:
- [ESP32_README.md](ESP32_README.md) - Complete hardware and software setup
- [config.h](config.h) - Main configuration file
- [Output_ESP32.cpp](Output_ESP32.cpp) - PWM motor control

### Original Project:
- ELECTRONOOBS Tutorial: http://www.electronoobs.com/eng_arduino_tut117.php
- MultiWii Wiki: http://www.multiwii.com/wiki

### ESP32 Resources:
- ESP32 Arduino Core: https://github.com/espressif/arduino-esp32
- ESP-NOW Guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

---

## Session Notes

### Methodology:
- Incremental changes with compilation verification after each step
- Platform-specific code properly guarded
- Safety prioritized over convenience
- Comprehensive documentation for future reference

### Tools Used:
- Arduino CLI for compilation
- Git for version control
- VSCode for development environment
- Claude Code (AI pair programming)

### Best Practices Followed:
- Read files before editing
- Test compilation between changes
- Use meaningful commit messages
- Document hardware requirements
- Include safety warnings
- Maintain backwards compatibility where possible

---

## Contact & Attribution

**Original Code:**
- MultiWii by Alexandre Dubus
- Brushed drone adaptation by iforced2D
- NRF24 version by ELECTRONOOBS (2020)

**ESP32 Adaptation:**
- Developer: jlmyra
- AI Assistant: Claude Sonnet 4.5
- Date: January 2026

**License:** GPL (inherited from MultiWii project)

---

*This document was generated to preserve the development session and assist future developers or the original developer when resuming work on this project.*
