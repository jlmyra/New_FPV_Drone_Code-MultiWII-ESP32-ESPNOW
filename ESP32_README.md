
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
You'll need a separate ESP32 board configured as a transmitter. The transmitter should send RC data in this structure:

```cpp
typedef struct __attribute__((packed)) struct_message {
  uint8_t throttle;  // 0-255
  uint8_t yaw;       // 0-255
  uint8_t pitch;     // 0-255
  uint8_t roll;      // 0-255
  uint8_t AUX1;      // 0 or 1
  uint8_t AUX2;      // 0 or 1
  uint8_t switches;  // Bit flags
} struct_message;
```

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
- **Frequency**: 32kHz (optimized for brushed motors)
- **Resolution**: 10-bit (0-1023)
- **Channels**: 0-3 for motors, 4 for buzzer

To change PWM frequency, edit `Output_ESP32.cpp`:
```cpp
#define PWM_FREQUENCY 32000  // Change this value
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

Default channel mapping (can be changed in `ESP_NOW_RX.cpp` line 105):
- Channel 1 (THROTTLE): 2000-1000 (inverted)
- Channel 2 (ROLL): 2000-1000 (inverted)
- Channel 3 (PITCH): 1000-2000 (normal)
- Channel 4 (YAW): 2000-1000 (inverted)
- Channel 5 (AUX1): 2000-1000
- Channel 6 (AUX2): 2000-1000

To invert a channel, swap the map() function parameters.

## Safety Features

### Failsafe
- If signal is lost for >1 second, all channels reset to safe values
- Throttle goes to 0
- All other channels center to 128 (1500µs equivalent)

### Arming
- Configured in `config.h`
- By default: throttle down + yaw right to arm
- Configure via `#define ALLOW_ARM_DISARM_VIA_TX_YAW`

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
