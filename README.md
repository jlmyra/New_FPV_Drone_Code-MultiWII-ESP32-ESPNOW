
# MultiWii Drone for ESP32 MH ET Live MiniKit with ESP-NOW

This is a modified version of the MultiWii brushed drone flight controller, adapted for the **ESP32 MH ET Live MiniKit** development board with **ESP-NOW** wireless communication.

## Hardware Requirements

### ESP32 Board
- **ESP32 MH ET Live MiniKit** (or compatible ESP32 DevKit)

### Sensors
- **MPU6050** - 6-axis IMU (Gyroscope + Accelerometer)
  - Connected via I2C
  - SDA: GPIO21
  - SCL: GPIO22
  - VCC: 3.3V
  - GND: GND
  - **AD0: Connect to GND** (sets I2C address to 0x68)

**IMPORTANT - MPU6050 AD0 Pin:**
The AD0 pin on the MPU6050 determines its I2C address:
- **AD0 → GND**: Address = 0x68 (default, used by this firmware)
- **AD0 → VCC**: Address = 0x69

If AD0 is left floating (unconnected), the address may be unstable and cause I2C communication failures. **Always connect AD0 to GND.**

### Motors (Brushed DC Motors)
- 4x Brushed DC motors with compatible ESCs
- Motor connections:
  - Motor 1 (Rear Right): GPIO13
  - Motor 2 (Front Right): GPIO25
  - Motor 3 (Rear Left): GPIO14
  - Motor 4 (Front Left): GPIO27

### Additional Components
- **Buzzer**: GPIO32 (optional but recommended)
- **LED**: GPIO33
- **Battery Voltage Monitor**: GPIO34 (ADC input)

### Buzzer Circuit (2N2222 Driver)

The ESP32 GPIO can only source ~12mA, which isn't enough for most buzzers. Use a 2N2222 NPN transistor to drive the buzzer:

**Schematic:**
```
        +3.3V (or +5V)
            │
            │
         [Buzzer]
            │
            ├───────|◁────────┐  (1N4148 flyback diode, cathode to +V)
            │                 │
            C                 │
      ┌─────┤ 2N2222          │
      │     B                 │
      │     │                 │
      │  [1kΩ]                │
      │     │                 │
      │  GPIO 32              │
      │                       │
      E                       │
      │                       │
     GND ─────────────────────┘
```

**Components:**
| Component | Value | Purpose |
|-----------|-------|---------|
| R1 | 1kΩ | Base current limiter |
| Q1 | 2N2222 | NPN switching transistor |
| D1 | 1N4148 | Flyback diode (optional, recommended for magnetic buzzers) |
| Buzzer | 3.3V-5V | Passive buzzer recommended (responds to 2kHz PWM) |

**2N2222 Pinout (TO-92 package, flat side facing you):**
```
  E  B  C
  │  │  │
  └──┴──┘
```
- **E** (Emitter, left) → GND
- **B** (Base, center) → 1kΩ resistor → GPIO 32
- **C** (Collector, right) → Buzzer negative terminal

**Notes:**
- Use a **passive buzzer** for best results - it will produce a 2kHz tone from the PWM signal
- Active buzzers work but the PWM is unnecessary (simple HIGH/LOW would suffice)
- The flyback diode protects the transistor from back-EMF (only needed for magnetic buzzers, not piezo)
- Buzzer positive terminal connects to +3.3V or +5V depending on buzzer rating

## Pin Configuration

| Component | GPIO Pin | Notes |
|-----------|----------|-------|
| MPU6050 SDA | 21 | I2C Data |
| MPU6050 SCL | 22 | I2C Clock |
| Motor 1 | 13 | PWM Output (LEDC Channel 0) - Safe pin |
| Motor 2 | 25 | PWM Output (LEDC Channel 1) - Safe pin |
| Motor 3 | 14 | PWM Output (LEDC Channel 2) - Safe pin |
| Motor 4 | 27 | PWM Output (LEDC Channel 3) - Safe pin |
| Buzzer | 32 | PWM Output (LEDC Channel 4) |
| LED | 33 | Digital Output - Safe pin |
| Battery Voltage | 34 | ADC1_CH6 (input only) |

## Motor Layout (QUAD X Configuration)

The motors are arranged in an X configuration with the following positions:

| Motor # | GPIO Pin | Physical Position | Rotation Direction |
|---------|----------|-------------------|-------------------|
| Motor 1 | 13 | Rear Right | CW (Clockwise) |
| Motor 2 | 25 | Front Right | CCW (Counter-Clockwise) |
| Motor 3 | 14 | Rear Left | CCW (Counter-Clockwise) |
| Motor 4 | 27 | Front Left | CW (Clockwise) |

### Visual Diagram (Top View):

```
            FRONT
              ↑

        M4 ⟲      ⟳ M2
         [27]      [25]
            \  ✈  /
            /     \
         [14]      [13]
        M3 ⟲      ⟳ M1

            REAR
```

**Legend:**
- ⟳ = Clockwise (CW) rotation
- ⟲ = Counter-Clockwise (CCW) rotation
- Numbers in brackets [ ] = GPIO pins

**Important:** Diagonal motors spin in the same direction. This configuration provides yaw control through differential thrust.

## Power Requirements

### CRITICAL: USB vs Battery Power

| Power Source | Max Current | Suitable For |
|--------------|-------------|--------------|
| **USB (5V)** | ~500mA | Configuration, IMU calibration, communication testing |
| **1S LiPo (3.7V)** | 5-10A+ | **Full motor operation (REQUIRED)** |

⚠️ **WARNING:** USB power alone (~500mA) **cannot** power the motors. Attempting to spin motors on USB will cause:
- ESP32 resets (brown-out detection triggered)
- Erratic behavior
- Potential damage to USB port

**Always connect a LiPo battery before arming the drone.**

### What Works on USB Power Only
- ✅ ESP-NOW communication testing
- ✅ IMU/gyro calibration
- ✅ MultiWiiConf GUI connection
- ✅ LED and buzzer testing
- ✅ Configuration changes
- ❌ Motor spinning (will reset ESP32)

### Motor Current Draw
4 brushed motors can draw **2-4A total** at full throttle, with **peak currents even higher** during startup. A 1S LiPo with at least 25C rating is recommended.

---

## Battery Voltage Monitoring (1S LiPo)

The drone uses GPIO34 (ADC1_CH6) for battery voltage monitoring. For a **1S LiPo battery (3.0V - 4.2V)**:

### **Option 1: Simple 2:1 Voltage Divider (Recommended)**

Since 4.2V exceeds the ESP32's safe ADC range (3.6V max with 11dB attenuation), use a simple voltage divider:

**Circuit:**
```
Battery+ ----[R1: 10kΩ]----+----[R2: 10kΩ]---- GND
                            |
                         GPIO34
```

**Components:**
- R1: 10kΩ resistor (1% tolerance preferred)
- R2: 10kΩ resistor (1% tolerance preferred)

**Result:**
- 4.2V battery → 2.1V at GPIO34 ✓ Safe!
- 3.0V battery → 1.5V at GPIO34 ✓ Safe!

### **Option 2: 11dB Attenuation Only (Risky)**

If you don't want a voltage divider, you can rely on the ESP32's 11dB attenuation, but:
- ⚠️ Max safe voltage: ~3.6V
- ⚠️ Full charge (4.2V) will exceed this and may damage the ADC
- ⚠️ Not recommended unless you strictly limit battery voltage to 3.5V

### **Calibration**

After building your voltage divider:

1. **Measure actual battery voltage** with a multimeter (e.g., 4.15V)
2. **Upload code and read the reported voltage** via serial monitor or configurator
3. **Calculate VBATSCALE:**
   ```
   VBATSCALE = (reported_value × 10) / actual_voltage
   ```
4. **Update config.h** line 925 with the new VBATSCALE value
5. **Re-upload and verify** the voltage matches your multimeter

**Default VBATSCALE:** 42 (may need adjustment based on your voltage divider)

### **Voltage Thresholds (1S LiPo)**

Update these values in [config.h](config.h) lines 927-929:
- **VBATLEVEL_WARN1:** 37 (3.7V - low battery warning)
- **VBATLEVEL_WARN2:** 35 (3.5V - very low battery)
- **VBATLEVEL_CRIT:** 33 (3.3V - critical, land immediately!)

**Note:** Values are in 0.1V units (e.g., 37 = 3.7V)

---

## I2C Wiring Best Practices

The MPU6050 communicates with the ESP32 via I2C. Proper wiring is critical for reliable gyro/accelerometer data.

### Do NOT Twist I2C Wires

Unlike differential signals (USB, Ethernet), I2C is **single-ended**. Twisting SDA and SCL together causes:
- **Capacitive coupling** between lines - clock edges induce noise on data
- **Crosstalk** - fast SCL transitions corrupt SDA readings
- **Increased capacitance** - slows signal edges, causes communication failures

### Recommended Wiring

1. **Keep wires short** - under 10cm is ideal for 400kHz I2C
2. **Run SDA and SCL parallel** with 1-2mm gap between them
3. **Include a ground wire** running alongside (reduces noise pickup)
4. **Route away from motor wires** and the ESP32 antenna area
5. **Use shielded cable** for longer runs (shield grounded at one end only)

### Pull-up Resistors

The MPU6050 module typically includes 4.7kΩ pull-ups. If using a bare chip:
- Add 4.7kΩ pull-up resistors from SDA and SCL to 3.3V
- For longer wires, use 2.2kΩ pull-ups

### Mounting the MPU6050

If mounting the IMU above the ESP32:
- **Vibration isolation** is critical - use foam tape or rubber standoffs
- **Heat** from the ESP32 can cause gyro drift - maximize airflow
- **EMI** from WiFi radio may affect readings - consider a ground plane between them

---

## PWM Frequency Tuning

The motor PWM frequency affects performance characteristics of brushed motors.

### Current Setting

```cpp
#define PWM_FREQUENCY 16000  // 16kHz - balanced setting
```

Edit in [Output_ESP32.cpp:16](Output_ESP32.cpp#L16)

### Frequency Trade-offs

| Frequency | Pros | Cons |
|-----------|------|------|
| **8kHz** | Maximum torque, best low-speed response | Audible whine, motor heating |
| **16kHz** | Good torque, minimal audible noise | Slight efficiency loss vs 8kHz |
| **32kHz** | Silent operation, less motor heating | Reduced low-speed torque |

### How It Works

Lower PWM frequencies allow higher **peak currents** during each pulse:
- More time for current to ramp up through motor inductance
- Higher peak current = stronger magnetic field = more torque
- Better responsiveness at low throttle (important for hover stability)

Higher frequencies spread current more evenly:
- Smoother motor operation
- Less audible noise (above human hearing at 20kHz+)
- Less efficient due to switching losses

### Recommendation

- **16kHz** - Best balance for most builds (current default)
- **8kHz** - If you need more low-end punch and don't mind motor whine
- **32kHz** - If noise is a priority and you have headroom on thrust

---

## Propeller Selection Guide

Choosing the right propeller affects flight characteristics significantly.

### Blade Count Comparison

| Blades | Efficiency | Thrust | Response | Noise |
|--------|------------|--------|----------|-------|
| **2-blade** | Best | Lower | Slower | Quietest |
| **3-blade** | Good | Medium | Medium | Medium |
| **4-blade** | Lower | Highest | Fastest | Loudest |

**2-blade props:**
- Most efficient (less blade interference)
- Smoother, quieter operation
- Best for longer flight times
- Slower response to throttle changes

**3-blade props:**
- Good compromise between efficiency and thrust
- Popular for freestyle/general use
- Moderate noise levels

**4-blade props:**
- Maximum thrust from given diameter
- Fastest throttle response
- Best for heavy builds or aggressive flying
- Shortest flight times, loudest

### Diameter vs Blade Count Example

Comparing **46mm 2-blade** vs **31mm 3-blade**:

| Property | 46mm 2-blade | 31mm 3-blade |
|----------|--------------|--------------|
| Disc area | 1662 mm² | 755 mm² |
| Static thrust | Higher | Lower |
| Efficiency | Better | Worse |
| Response | Slower | Faster |
| Motor load | Higher | Lower |
| Noise | Quieter | Higher pitch |

**46mm 2-blade** advantages:
- 2.2x more disc area = more air moved = more lift
- Better hover efficiency
- Quieter at same thrust level

**31mm 3-blade** advantages:
- Lower rotational inertia = faster RPM changes
- Snappier response for acrobatic flying
- Less stress on motors (lower torque required)
- Fits in tighter spaces

### Recommendation for This Build

For a brushed micro quad:
- **Larger 2-blade** (40-55mm) - Best for efficiency and flight time
- **Smaller 3-blade** (31-40mm) - Better for indoor flying and quick maneuvers

Match prop size to your motor's recommended range. Oversized props will overheat motors; undersized props waste motor capability.

---

## Software Setup

### 1. Install Arduino IDE
Download and install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)

### 2. Install ESP32 Board Support

**Required Version:** ESP32 Arduino Core **3.0.7** (or compatible 3.x version)

1. Open Arduino IDE
2. Go to File > Preferences
3. Add this URL to "Additional Board Manager URLs":
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Go to Tools > Board > Boards Manager
5. Search for "ESP32" and install "ESP32 by Espressif Systems" **version 3.0.7**

**Important:** This code uses ESP32 Core 3.x API features:
- `ledcAttach(pin, frequency, resolution)` - newer API
- `ledcWrite(pin, duty_cycle)` - uses pin numbers instead of channels
- If using Core 2.x, you'll need to modify the PWM code in Output_ESP32.cpp

### 3. Install Required Libraries
Go to Tools > Manage Libraries and install:
- **Wire** (usually pre-installed with ESP32)
- **WiFi** (included with ESP32 board support)

### 4. Select Board Settings
1. Tools > Board > ESP32 Arduino > **ESP32 Dev Module**
2. Tools > Upload Speed > **115200** (or higher if stable)
3. Tools > CPU Frequency > **240MHz** (recommended for flight controller)
4. Tools > Flash Frequency > **80MHz**
5. Tools > Flash Size > **4MB (32Mb)**
6. Tools > Partition Scheme > **Default 4MB with spiffs**
7. Tools > Port > Select your ESP32's COM port

## ESP-NOW Wireless Communication

### What is ESP-NOW?
ESP-NOW is a connectionless Wi-Fi communication protocol developed by Espressif. It provides:
- Low latency (1-10ms typical)
- No need for Wi-Fi router
- Up to 250-byte payloads
- Low power consumption
- Simple peer-to-peer communication

### Transmitter Setup
You'll need a separate ESP32 board configured as a transmitter. Two transmitter variants are available:

1. **NEW_FPV_Transmitter_Code_ESP32_ESPNOW** - Standard spring-return joysticks
2. **Transmitter_ESP32_No_Spring_Joy** - No-spring throttle (linear mapping)

The transmitter must send RC data in this structure (7 bytes):

```cpp
typedef struct __attribute__((packed)) struct_message {
  uint8_t throttle;  // 0-255, mapped to 1000-2000µs
  uint8_t yaw;       // 0-255, mapped to 2000-1000µs (inverted)
  uint8_t pitch;     // 0-255, mapped to 1000-2000µs
  uint8_t roll;      // 0-255, mapped to 2000-1000µs (inverted)
  uint8_t AUX1;      // 0 or 1, flight mode switch
  uint8_t AUX2;      // 0 or 1, secondary function
  uint8_t switches;  // Bit flags: bit0=left btn, bit1=right btn
} struct_message;
```

The drone sends telemetry back (11 bytes):

```cpp
typedef struct __attribute__((packed)) struct_ack {
  uint8_t vbat;       // Battery voltage (0.1V units, 37 = 3.7V)
  uint8_t rssi;       // Signal strength (0-100%)
  int16_t heading;    // Compass heading (degrees)
  int16_t pitch;      // Pitch angle (0.1 degrees)
  int16_t roll;       // Roll angle (0.1 degrees)
  int16_t alt;        // Altitude (cm)
  uint8_t flags;      // bit0=armed, bit1=angle, bit2=horizon, bit3=baro
} struct_ack;
```

**CRITICAL:** Both structures must use `__attribute__((packed))` to ensure binary compatibility between transmitter and receiver.

### Getting Your ESP32 MAC Address
When you first upload and run the code:
1. Open Serial Monitor (Tools > Serial Monitor)
2. Set baud rate to **115200**
3. The MAC address will be printed: `ESP32 Receiver MAC Address: XX:XX:XX:XX:XX:XX`
4. Use this MAC address to pair your transmitter

## Configuration

### Main Configuration File
Edit `config.h` to customize your drone:

1. **Frame Type** (Line 39):
   ```cpp
   #define QUADX  // For X-configuration quadcopter
   ```

2. **Motor Settings** (Lines 63-72):
   ```cpp
   #define MINTHROTTLE 1000
   #define MAXTHROTTLE 2000
   #define MINCOMMAND  1000
   ```

3. **PID Tuning**: Adjust PID values for stable flight

### ESP32-Specific Settings
Pin definitions are in `config.h` starting at line 501:
```cpp
#define ESP32_I2C_SDA 21
#define ESP32_I2C_SCL 22
#define ESP32_MOTOR1_PIN 13
#define ESP32_MOTOR2_PIN 25
#define ESP32_MOTOR3_PIN 14
#define ESP32_MOTOR4_PIN 27
#define ESP32_BUZZER_PIN 32
// etc...
```

## PWM Configuration

The ESP32 uses **LEDC (LED Controller)** for PWM generation:
- **Frequency**: 16kHz (optimized for brushed motors - see [PWM Frequency Tuning](#pwm-frequency-tuning) for details)
- **Resolution**: 10-bit (0-1023)
- **Channels**: 0-3 for motors, 4 for buzzer

To change PWM frequency, edit [Output_ESP32.cpp:16](Output_ESP32.cpp#L16):
```cpp
#define PWM_FREQUENCY 16000  // 8kHz=more torque, 32kHz=quieter
```

## Uploading the Code

1. Connect ESP32 to your computer via USB
2. Select the correct COM port (Tools > Port)
3. Press and hold the BOOT button on ESP32 (if upload fails)
4. Click Upload button in Arduino IDE
5. Wait for "Hard resetting via RTS pin..." message
6. Open Serial Monitor to see initialization messages

## Troubleshooting

### Motors Not Spinning
- Check motor connections to correct GPIO pins
- Verify motor power supply is connected
- Check PWM frequency setting
- Ensure ESCs are compatible with signal

### IMU Not Responding
- Check I2C connections (SDA=21, SCL=22)
- Verify MPU6050 is powered with 3.3V (NOT 5V!)
- Run I2C scanner to find MPU6050 address
- Check pull-up resistors on I2C lines

### ESP-NOW Connection Issues
- Verify both ESP32 boards are using same Wi-Fi channel
- Check MAC address is correctly set in transmitter
- Ensure transmitter is sending data in correct format
- Check Serial Monitor for "ESP-NOW Initialized Successfully"

### Compilation Errors
- Ensure ESP32 board support is installed correctly
- Check that all header files are in the same folder
- Verify config.h has `#define ESP_NOW_RX` enabled
- Make sure you selected "ESP32 Dev Module" as board

## Channel Mapping

Default channel mapping (defined in `ESP_NOW_RX.cpp`):

| Channel | Name | Range | Source |
|---------|------|-------|--------|
| 1 | THROTTLE | 1000-2000 | Left stick Y |
| 2 | YAW | 2000-1000 (inverted) | Left stick X |
| 3 | PITCH | 1000-2000 | Right stick Y |
| 4 | ROLL | 2000-1000 (inverted) | Right stick X |
| 5 | AUX1 | 1000/2000 | Toggle switch 1 |
| 6 | AUX2 | 1000/2000 | Toggle switch 2 |
| 7 | AUX3 | 1000/2000 | Left joystick button |
| 8 | AUX4 | 1000/2000 | Right joystick button |

**Joystick Buttons:** The `switches` byte from the transmitter is mapped to AUX3/AUX4:
- bit0 (left button) → AUX3
- bit1 (right button) → AUX4 (can be used for BOXBEEPERON)

To invert a channel, swap the map() function parameters.

## Safety Features

### Failsafe
- If signal is lost for >1 second, all channels reset to safe values
- Throttle goes to 0
- All other channels center to 128 (1500µs equivalent)

### Arming and Disarming

**Arming Thresholds:**
- MINCHECK = 1100 (stick considered "low" below this)
- MAXCHECK = 1900 (stick considered "high" above this)
- Stick values are mapped from 0-255 (transmitter) to 1000-2000 (RC PWM)

**To ARM the drone:**
1. Move the **left joystick to the bottom-right corner**
2. Hold for ~0.4 seconds (20 cycles)
3. This means: Throttle DOWN (low) + Yaw RIGHT (high)
4. LED pattern will change and buzzer may beep to indicate armed state

**To DISARM the drone:**
1. Move the **left joystick to the bottom-left corner**
2. Hold for ~0.4 seconds
3. This means: Throttle DOWN (low) + Yaw LEFT (low)

**Why both DOWN and RIGHT?**
The joystick has two independent axes (two potentiometers):
- Y-axis (vertical) controls Throttle
- X-axis (horizontal) controls Yaw

Moving to the corner activates both axes simultaneously - they're independent measurements.

**Configuration:**
- Configured in `config.h`
- Default: `#define ALLOW_ARM_DISARM_VIA_TX_YAW` (throttle down + yaw)
- Alternative: `#define ALLOW_ARM_DISARM_VIA_TX_ROLL` (throttle down + roll)

**Safety:** `ONLYARMWHENFLAT` prevents arming when the drone is tilted.

## Performance Notes

- **Loop Time**: ~2800µs (configured in config.h)
- **ESP-NOW Latency**: 1-10ms typical
- **I2C Speed**: 400kHz
- **CPU Speed**: 240MHz recommended

## File Structure

- **MultiWii_RF24.ino** - Main sketch file
- **config.h** - Main configuration file
- **ESP_NOW_RX.cpp/.h** - ESP-NOW receiver implementation
- **Output_ESP32.cpp** - ESP32 LEDC PWM output
- **Sensors.cpp** - Updated with ESP32 I2C support
- **MultiWii.cpp** - Flight controller main loop
- **RX.cpp** - Updated to support ESP-NOW

## Original Project Credits

- Original MultiWii by Alexandre Dubus
- Modified for brushed drone by iforced2D
- NRF24 version by ELECTRONOOBS
- ESP32 adaptation (2026)

## License

This project is licensed under GPL (GNU General Public License) as inherited from the original MultiWii project.

## Support & Resources

- Original Tutorial: http://www.electronoobs.com/eng_arduino_tut117.php
- MultiWii Wiki: http://www.multiwii.com/wiki
- ESP32 Documentation: https://docs.espressif.com/
- ESP-NOW Guide: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

## Important Notes

⚠️ **SAFETY WARNING**:
- Always remove propellers when testing on a bench
- Test in a safe environment away from people and obstacles
- Start with low throttle and gradually increase
- Have an emergency cutoff plan
- Never fly over people or near airports
- Follow local drone regulations

📝 **Before First Flight**:
1. Calibrate IMU (level surface required)
2. Test all motor directions
3. Verify channel directions match your transmitter
4. Test failsafe behavior
5. Tune PID values for your specific setup
6. Perform extensive ground testing

## Version History

- **2026** - ESP32 MH ET Live MiniKit version with ESP-NOW
- **2020** - ELECTRONOOBS NRF24 version
- **Original** - MultiWii brushed drone by iforced2D
