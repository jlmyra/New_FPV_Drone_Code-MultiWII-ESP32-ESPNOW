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

## Session: January 20-21, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Verified ESP-NOW communication between transmitter and drone, documented arming procedures, and resolved MultiWiiConf GUI issues on macOS.

### Verified Working

#### 1. ESP-NOW Communication Confirmed

**Test Results:**
- Transmitter LCD shows telemetry when drone powers on: `23.8V 78% DIS`
- Second row shows: `T:1 M:ACRO`
- Communication is bidirectional (drone sends telemetry back)

**Note:** Voltage reading (23.8V) is incorrect because no battery is connected. The drone is powered via USB for bench testing. GPIO 34 ADC reads floating/noise without battery connected - this is expected behavior.

#### 2. Arming Procedure Documented

**Arming Thresholds (from [MultiWii.h](MultiWii.h)):**
- MINCHECK = 1100 (stick values below this = "low")
- MAXCHECK = 1900 (stick values above this = "high")

**Arm Command:** Throttle LOW + Yaw HIGH (bottom-right corner of left stick)
- Hold for ~0.4 seconds (20 cycles × 20ms)
- Stick positions defined in [MultiWii.cpp:920-993](MultiWii.cpp#L920-L993)

**Disarm Command:** Throttle LOW + Yaw LOW (bottom-left corner of left stick)

**Joystick Explanation:**
The left joystick has two independent axes (two potentiometers):
- Y-axis (vertical) = Throttle → GPIO 39
- X-axis (horizontal) = Yaw → GPIO 36

Moving to the bottom-right corner means:
- Throttle stick pushed DOWN (Y-axis low) → throttle < MINCHECK
- Yaw stick pushed RIGHT (X-axis high) → yaw > MAXCHECK

Both conditions are satisfied simultaneously because they're independent axes.

### Issues Resolved

#### 1. Serial Port Busy Error (MultiWiiConf)

**Symptom:** Java exception "Port busy" when connecting via Processing GUI

**Cause:** Arduino Serial Monitor was holding `/dev/cu.usbserial-0001`

**Solution:** Close Arduino Serial Monitor (or kill process) before using MultiWiiConf

#### 2. controlP5 Library Crash Fix

**Symptom:** `IndexOutOfBoundsException` when clicking dropdowns in MultiWiiConf

**Root Cause:** `ListBox.onRelease()` in controlP5 2.2.6 doesn't validate index before accessing items

**Fix Applied:** Added bounds checking to `/Users/jlmyracle/Documents/Processing/libraries/controlP5/src/controlP5/ListBox.java:135`

```java
if ( index < 0 || index >= items.size( ) ) {
    return;
}
```

**Patch Log:** See `/Users/jlmyracle/Documents/Processing/libraries/controlP5/PATCH_LOG.md`

#### 3. Retina Display Fix (MultiWiiConf)

**Symptom:** Graphics overlapping and illegible on macOS Retina displays

**Fix:** Added `pixelDensity(1)` to `MultiWiiConf.pde:394`

#### 4. ListBox Event Handler Fix (MultiWiiConf)

**Symptom:** Clicking serial port in dropdown visually selects item but doesn't trigger connection

**Root Cause:** In controlP5 2.2.6, ListBox is a Controller (not ControllerGroup), so `isGroup()` always returns false

**Fix:** Changed event handler in `MultiWiiConf.pde:2597-2605` to check `isController()` and use `.equals()` for string comparison

### USB Power Testing Notes

When running the drone from USB (no battery):
- Motors will NOT spin (USB can't supply enough current)
- Communication, IMU, and configuration can be tested
- Battery voltage reading will be incorrect (floating ADC)
- LED and buzzer will function normally

### Transmitter LCD Display Reference

| Display | Meaning |
|---------|---------|
| First row `23.8V 78% DIS` | Battery voltage, RSSI %, ARM status |
| Second row `T:1 M:ACRO` | Throttle value, Flight mode |
| `ARM` | Drone is armed (motors will respond) |
| `DIS` | Drone is disarmed (safe) |
| `ACRO` | Acrobatic mode (no stabilization) |
| `STAB` | Stabilized/Angle mode (self-leveling) |

---

## Session: January 21, 2026 (Arming Fix)

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Fixed arming issue caused by inverted YAW and ROLL channel mappings on the receiver side.

### Problem

Drone would not arm despite trying all joystick positions. User moved left stick through full 360° travel holding positions for 1 second each - no arming occurred.

### Root Cause

In `ESP_NOW_RX.cpp`, the YAW and ROLL channels were being inverted during the mapping from transmitter values (0-255) to RC values (1000-2000). The transmitter already handles axis inversion via `*Reverse` settings, so the receiver was double-inverting these channels.

**Original (incorrect):**
```cpp
espnow_rcData[YAW] =  map(MyData.yaw,  0, 255, 2000, 1000);  // Inverted!
espnow_rcData[ROLL] = map(MyData.roll, 0, 255, 2000, 1000);  // Inverted!
```

**Fixed:**
```cpp
espnow_rcData[YAW] =  map(MyData.yaw,  0, 255, 1000, 2000);  // Direct mapping
espnow_rcData[ROLL] = map(MyData.roll, 0, 255, 1000, 2000);  // Direct mapping
```

### Files Modified

- `ESP_NOW_RX.cpp:176,178` - Fixed channel mapping direction

### Result

Arming now works. User successfully armed the drone using the left stick.

### Note on Stick Direction

The user reported arming with "upper right" stick position. This indicated the transmitter's `throttleReverse` setting was incorrect - changed from `true` to `false` in `Transmitter_ESP32_No_Spring_Joy.ino:45`. Arming now works with the documented "bottom-right" gesture.

### Debug Feature Added

Added `DEBUG_ESPNOW_ARM` flag (default false) to `ESP_NOW_RX.cpp:12` for future troubleshooting. When enabled, prints RC values and arming-related flags to Serial Monitor every 500ms.

### LED Behavior Summary

| State | LED (GPIO 33) |
|-------|---------------|
| Disarmed + ACC Calibrated | OFF |
| Armed | ON (solid) |
| Gyro calibrating at boot | Blinking |
| EEPROM checksum error | 6 blinks |
| Permanent log error | 9 blinks |

---

## Session: January 24, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Debugged motor PWM issue where Motor 3 (rear left) was not producing a signal. Tested multiple GPIO pins and added PWM initialization debug output.

### Problem

3 of 4 motors spin correctly. Motor 3 (rear left) produces no PWM signal as verified with oscilloscope.

### Debugging Steps

#### 1. GPIO 14 (Original Pin) - FAILED

**Symptom:** No PWM signal on GPIO 14

**Analysis:** GPIO 14 is a strapping pin that controls internal flash voltage during boot. May cause issues with PWM initialization.

#### 2. GPIO 26 - FAILED

Changed Motor 3 from GPIO 14 to GPIO 26. Still no signal.

#### 3. GPIO 16 - TESTING

Changed Motor 3 to GPIO 16. Added debug output to verify LEDC initialization.

### Files Modified

- `config.h:509` - Changed `ESP32_MOTOR3_PIN` from 14 → 26 → 16
- `Output_ESP32.cpp:31-53` - Added debug output to `initOutput()` to show ledcAttach success/failure for each motor

### Debug Output Added

```cpp
Serial.println("Initializing motor PWM...");
for (uint8_t i = 0; i < 4; i++) {
  Serial.print("  Motor "); Serial.print(i+1);
  Serial.print(" on GPIO "); Serial.print(ESP32_PWM_PIN[i]);
  bool success = ledcAttach(ESP32_PWM_PIN[i], PWM_FREQUENCY, PWM_RESOLUTION);
  Serial.print(" - ledcAttach: "); Serial.println(success ? "OK" : "FAILED");
  ledcWrite(ESP32_PWM_PIN[i], 0);
}
```

### Current Motor Pin Configuration

| Motor | Position | GPIO | Status |
|-------|----------|------|--------|
| Motor 1 | Rear Right | 13 | Working |
| Motor 2 | Front Right | 25 | Working |
| Motor 3 | Rear Left | 16 | Testing |
| Motor 4 | Front Left | 27 | Working |

### ESP32 GPIO Notes

**Strapping Pins (avoid for outputs):**
- GPIO 0, 2, 12, 15 - Boot mode selection
- GPIO 14 - Flash voltage (caused PWM issues)

**Input-Only Pins:**
- GPIO 34, 35, 36, 39

**Safe Output Pins:**
- GPIO 4, 5, 13, 16, 17, 18, 19, 23, 25, 26, 27, 32, 33

### Transmitter Axis Inversion Fix

Also fixed arming gesture - drone was arming with "up-left" instead of "down-right":

**Changes to Transmitter_ESP32_No_Spring_Joy.ino:45-46:**
```cpp
bool throttleReverse = true;   // Stick UP = low voltage, so invert
bool yawReverse      = false;  // Stick RIGHT = high voltage, no invert
```

### Next Steps

1. Upload code and check Serial Monitor for ledcAttach results
2. If Motor 3 shows "FAILED", investigate LEDC channel conflicts
3. If Motor 3 shows "OK" but still no signal, check writeMotors() function

---

## Session: January 26, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Fixed critical throttle lookup table bug, resolved gyro calibration issues, and successfully tested motor response. Identified power supply limitations when running from USB.

### Issues Resolved

#### 1. Lookup Table Empty After Boot (CRITICAL)

**Symptom:** Motors would not respond despite being armed. `rcCommand[THROTTLE]` always showed 0 regardless of stick position.

**Root Cause:** The `lookupThrottleRC[]` array was all zeros after boot. This lookup table converts throttle input to motor output. On ESP32, the EEPROM read timing differs from AVR, causing the lookup table computation to run before EEPROM data was loaded.

**Fix:** Added lookup table rebuild check in `MultiWii.cpp:708-724` that detects empty table and rebuilds it:

```cpp
// ESP32 FIX: Rebuild lookup table if empty (EEPROM timing issue)
if (lookupThrottleRC[5] == 0 && conf.minthrottle > 0) {
  Serial.println("*** Lookup table empty - rebuilding ***");
  // ... rebuild logic using same formula as Config_EEPROM.cpp
}
```

**Files Modified:**
- `MultiWii.cpp:708-724` - Added lookup table rebuild after `readEEPROM()`

**Verified Working:** Debug output shows rebuilt table:
```
lookupThrottleRC: 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 2000
```

#### 2. Motor and PID Debug Output Added

**Problem:** Difficult to diagnose why motors weren't responding

**Solution:** Added debug output to `writeMotors()` showing motor values, armed state, throttle, rcCommand, and PID values.

**Files Modified:**
- `Output_ESP32.cpp:66-92` - Added `DEBUG_MOTORS` flag and debug output

**Sample Output:**
```
Motors: 1186 1786 1786 1186  Armed:1 Thr:1538 rcCmd:1486 PID R:0 P:0 Y:-300
```

#### 3. Gyro Calibration Debug Added

**Problem:** LED kept blinking indicating gyro calibration never completing, PID values were extremely large (R:2211 P:-2211)

**Root Cause:** User had twisted I2C wires together, causing electrical interference and saturated gyro readings (16383, -16384, -16384).

**Solution:** Added debug output to track gyro calibration progress.

**Files Modified:**
- `Sensors.cpp:250-257` - Added calibration countdown and gyroADC value logging

**Note:** The twisted I2C wires should be separated. While calibration eventually completed, the gyro data quality may be compromised with twisted wires.

### ESP32 Reset During Motor Operation

**Symptom:** After successful arming and motor spin-up, the ESP32 reset (showing `ets Jul 29 2019 12:21:46` boot message).

**Likely Causes:**
1. **Insufficient USB power** - USB provides ~500mA max, but 4 brushed motors can draw 2-4A total
2. **Brown-out detection** - ESP32 monitors voltage and resets if it drops below threshold
3. **Current spikes** - Motor startup draws peak current exceeding supply capacity

**Solution:** **Use a LiPo battery** for motor testing. USB power is only suitable for:
- Configuration and communication testing
- IMU calibration
- LED and buzzer testing
- NOT for spinning motors

### Power Requirements

| Power Source | Max Current | Suitable For |
|--------------|-------------|--------------|
| USB (5V) | ~500mA | Configuration, IMU, communication |
| 1S LiPo (3.7V) | 5-10A+ | Full motor operation |

**Important:** Always connect a battery before attempting to arm and spin motors. USB power will cause resets or erratic behavior.

### Final Test Results

Before the power-related reset, the system demonstrated full functionality:

```
Motors: 1186 1786 1786 1186  Armed:1 Thr:1538 rcCmd:1486 PID R:0 P:0 Y:-300
Motors: 1270 1870 1870 1270  Armed:1 Thr:1614 rcCmd:1570 PID R:0 P:0 Y:-300
Motors: 1314 1914 1914 1314  Armed:1 Thr:1653 rcCmd:1614 PID R:0 P:0 Y:-300
```

- Arming works (Armed:1)
- Throttle responds (Thr values increase with stick)
- rcCommand calculates correctly (rcCmd follows Thr)
- Motors respond asymmetrically due to PID Y:-300 (expected with yaw input)

### Transmitter Axis Configuration (Final)

After multiple iterations, the correct settings for the HW-504 joysticks are:

```cpp
#define PIN_THROTTLE  36  // Left stick X-axis (horizontal = throttle)
#define PIN_YAW       39  // Left stick Y-axis (vertical = yaw)

bool throttleReverse = true;   // Inverted for correct direction
bool yawReverse      = true;   // Inverted for correct direction
bool pitchReverse    = true;
bool rollReverse     = true;
```

**Note:** The actual reverse settings may vary based on individual joystick wiring. Use `DEBUG_JOYSTICK true` in the transmitter code to verify raw ADC values match stick movement.

### Remaining Hardware Tasks

1. **Separate I2C wires** - Untwist SDA/SCL wires to improve gyro data quality
2. **Connect LiPo battery** - Required for motor testing
3. ~~**Verify Motor 3**~~ - **FIXED** (see below)
4. **First hover test** - With battery power and verified gyro

---

## Session: January 26, 2026 (Motor 3 Pin Fix)

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Fixed Motor 3 PWM issue by changing from GPIO 16 to GPIO 14. All 4 motors now operational.

### Problem

Motor 3 (Rear Left) was not receiving a PWM signal despite `ledcAttach` appearing to succeed.

### Root Cause

**GPIO 16** is problematic on ESP32:
- Connected to PSRAM clock on modules with PSRAM
- Used by internal flash on some ESP32 variants
- Can cause LEDC PWM conflicts or failures

### Fix

Changed Motor 3 pin from GPIO 16 to GPIO 14 in `config.h:509`:

```cpp
// Before:
#define ESP32_MOTOR3_PIN 16  // Motor 3 (left) - changed from 14, 26 (PWM issues)

// After:
#define ESP32_MOTOR3_PIN 14  // Motor 3 (left) - GPIO14 is safe for PWM
```

### GPIO 14 Safety Note

GPIO 14 (MTMS) is a strapping pin used for JTAG debugging, but it's safe for PWM output after boot completes. It has an internal pull-up and won't affect normal boot operation.

### Final Motor Pin Configuration

| Motor | Position | GPIO | Status |
|-------|----------|------|--------|
| 1 | Rear Right | 13 | Working |
| 2 | Front Right | 25 | Working |
| 3 | Rear Left | **14** | **Fixed** |
| 4 | Front Left | 27 | Working |

### Result

All 4 motors now respond to throttle commands. Ready for battery-powered hover testing.

---

## Future Considerations

- ADC calibration routine for battery accuracy
- ESP32-S3 support
- OTA updates via ESP-NOW
- Blackbox logging to SPIFFS
- Joystick calibration wizard with EEPROM storage
- Brown-out detection configuration for motor loads
