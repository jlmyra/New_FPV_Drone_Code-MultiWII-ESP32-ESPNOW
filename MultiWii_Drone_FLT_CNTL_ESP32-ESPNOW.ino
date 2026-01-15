
/*  From iforced2D, changed by ELECTRONOOBS 11/04/2020
 *  Modified for ESP32 MH ET Live MiniKit with ESP-NOW - 2026
 *
 *  === ORIGINAL ARDUINO VERSION ===
 *  This is the code used for a brushed drone. Motors are connected to D3, D5, D6 and D9
 *  NRF24 is connected to D13, D12, D11, D7 and D10
 *  IMU is connected to A4 and A5
 *  Buzzer is connected on D8
 *  Check the tutorial here: http://www.electronoobs.com/eng_arduino_tut117.php
 *  Schematic here: http://www.electronoobs.com/eng_arduino_tut117_sch1.php
 *  And the video here: https://youtu.be/J0x4ChjUS00
 *
 *  === ESP32 VERSION ===
 *  Motors connected to: GPIO13, GPIO25, GPIO14, GPIO27 (using LEDC PWM)
 *  ESP-NOW for wireless communication (no need for NRF24 module)
 *  IMU (MPU6050) connected to: GPIO21 (SDA), GPIO22 (SCL)
 *  Buzzer connected to: GPIO32
 *  Battery voltage monitoring: GPIO34 (ADC)
 *  LED: GPIO33
 *
 *  Go to ESP_NOW_RX.cpp to configure ESP-NOW communication
 *  Go to config.h to change drone configuration
 *  Go to Output_ESP32.cpp to change PWM signal values
 *  To invert channels go to ESP_NOW_RX.cpp line 105
 *  ESP32 pin definitions are in config.h starting at line 501
 *
 * Welcome to MultiWii.
 *
 * If you see this message, chances are you are using the Arduino IDE. That is ok.
 * To get the MultiWii program configured for your copter, you must switch to the tab named 'config.h'.
 * Maybe that tab is not visible in the list at the top, then you must use the drop down list at the right
 * to access that tab. In that tab you must enable your board or sensors and optionally various features.
 * For more info go to http://www.multiwii.com/wiki/index.php?title=Main_Page
 *
 * Have fun, and do not forget MultiWii is made possible and brought to you under the GPL License.
 *
 */

#if !defined(ESP32)
  // Original Arduino libraries for NRF24
  #include <SPI.h>
  #include <nRF24L01.h>
  #include <RF24.h>
#else
  // ESP32 libraries
  #include <esp_now.h>
  #include <WiFi.h>
  #include <Wire.h>
  #include <EEPROM.h>
#endif
