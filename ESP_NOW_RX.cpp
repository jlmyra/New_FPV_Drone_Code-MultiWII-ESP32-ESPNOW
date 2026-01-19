
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "ESP_NOW_RX.h"

#if defined(ESP_NOW_RX)

#include <esp_now.h>
#include <WiFi.h>

int16_t espnow_rcData[RC_CHANS];

// Structure to receive data - must match transmitter structure
typedef struct __attribute__((packed)) struct_message {
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
  uint8_t AUX1;
  uint8_t AUX2;
  uint8_t switches;
} struct_message;

struct_message MyData;

// Structure to send telemetry data back to transmitter
typedef struct __attribute__((packed)) struct_ack {
  uint8_t vbat;       // Battery voltage in 0.1V units (e.g., 37 = 3.7V)
  uint8_t rssi;       // Signal strength (0-100%)
  int16_t heading;    // Compass heading in degrees
  int16_t pitch;      // Pitch angle in 0.1 degrees
  int16_t roll;       // Roll angle in 0.1 degrees
  int16_t alt;        // Altitude in cm
  uint8_t flags;      // Status flags: bit0=armed, bit1=angle, bit2=horizon, etc.
} struct_ack;

struct_ack espnowAckPayload;

// Transmitter peer info for sending telemetry
static esp_now_peer_info_t txPeerInfo;
static uint8_t txMacAddress[6] = {0};
static bool txPeerAdded = false;

static unsigned long lastRecvTime = 0;
static bool dataReceived = false;
static float smoothedRssi = 0; // Will be set to actual value on first packet
static bool rssiInitialized = false;
static unsigned long lastTelemetryTime = 0;

// Failsafe timeout in milliseconds
#define ESPNOW_FAILSAFE_TIMEOUT 500

// Telemetry rate (10Hz = every 100ms, matches transmitter LCD update rate)
#define TELEMETRY_INTERVAL_MS 100

void resetESPNOWData()
{
  MyData.throttle = 0;
  MyData.yaw = 128;
  MyData.pitch = 128;
  MyData.roll = 128;
  MyData.AUX1 = 0;
  MyData.AUX2 = 0;
  MyData.switches = 0;
}

void resetESPNOWAckPayload()
{
  espnowAckPayload.vbat = 0;
  espnowAckPayload.rssi = 0;
  espnowAckPayload.heading = 0;
  espnowAckPayload.pitch = 0;
  espnowAckPayload.roll = 0;
  espnowAckPayload.alt = 0;
  espnowAckPayload.flags = 0;
}

// Callback function executed when data is received
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (len != sizeof(MyData)) return; // Safety check: ignore packets of wrong size
  memcpy(&MyData, incomingData, sizeof(MyData));
  dataReceived = true;
  lastRecvTime = millis();

  // Store RSSI for signal quality reporting
  // Initialize with first actual reading, then use EMA for smoothing
  if (!rssiInitialized) {
    smoothedRssi = recv_info->rx_ctrl->rssi;
    rssiInitialized = true;
  } else {
    smoothedRssi = (smoothedRssi * 0.9) + (recv_info->rx_ctrl->rssi * 0.1);
  }

  // Add transmitter as peer if not already added (for sending telemetry back)
  if (!txPeerAdded) {
    memcpy(txMacAddress, recv_info->src_addr, 6);
    memcpy(txPeerInfo.peer_addr, txMacAddress, 6);
    txPeerInfo.channel = 0;
    txPeerInfo.encrypt = false;

    if (esp_now_add_peer(&txPeerInfo) == ESP_OK) {
      txPeerAdded = true;
      Serial.print("Transmitter paired: ");
      for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", txMacAddress[i]);
        if (i < 5) Serial.print(":");
      }
      Serial.println();
    }
  }
}

void ESPNOW_Init() {
  resetESPNOWData();
  resetESPNOWAckPayload();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Print MAC address for pairing with transmitter
  Serial.print("ESP32 Receiver MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving data
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW Initialized Successfully");
}

void ESPNOW_Read_RC() {

  unsigned long now = millis();

  // Check if data was lost (failsafe after 500ms)
  if (now - lastRecvTime > ESPNOW_FAILSAFE_TIMEOUT) {
    // Signal lost - reset to safe values
    resetESPNOWData();
  }

  // Prepare telemetry payload with current drone status
  espnowAckPayload.vbat = analog.vbat;  // Already in 0.1V units from MultiWii
  espnowAckPayload.rssi = map(constrain((int)smoothedRssi, -100, -30), -100, -30, 0, 100);  // Convert dBm to 0-100%
  espnowAckPayload.heading = att.heading;
  espnowAckPayload.pitch = att.angle[PITCH];
  espnowAckPayload.roll = att.angle[ROLL];
  espnowAckPayload.alt = constrain(alt.EstAlt, -32768, 32767);  // Clamp to int16 range
  espnowAckPayload.flags = 0;
  if (f.ARMED) espnowAckPayload.flags |= 0x01;
  if (f.ANGLE_MODE) espnowAckPayload.flags |= 0x02;
  if (f.HORIZON_MODE) espnowAckPayload.flags |= 0x04;
  if (f.BARO_MODE) espnowAckPayload.flags |= 0x08;

  // Send telemetry back to transmitter at 10Hz (if paired)
  // This matches the transmitter's LCD update rate and reduces ESP-NOW traffic
  if (txPeerAdded && (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)) {
    lastTelemetryTime = now;
    esp_now_send(txMacAddress, (uint8_t *)&espnowAckPayload, sizeof(espnowAckPayload));
  }

  // Map received data to RC channels
  // Standard mapping: Left stick = Throttle/Yaw, Right stick = Pitch/Roll
  // If your channels are inverted, reverse the map values (swap 1000/2000)
  espnow_rcData[THROTTLE] = map(MyData.throttle, 0, 255, 1000, 2000);
  espnow_rcData[YAW] =      map(MyData.yaw,      0, 255, 2000, 1000);
  espnow_rcData[PITCH] =    map(MyData.pitch,    0, 255, 1000, 2000);
  espnow_rcData[ROLL] =     map(MyData.roll,     0, 255, 2000, 1000);

  // AUX switches: TX sends 1 when switch is ON (due to !digitalRead with INPUT_PULLUP)
  // Map: 1 (ON) -> 2000 (high), 0 (OFF) -> 1000 (low)
  espnow_rcData[AUX1] =     map(MyData.AUX1,     0, 1, 1000, 2000);
  espnow_rcData[AUX2] =     map(MyData.AUX2,     0, 1, 1000, 2000);

  // Map joystick buttons from switches byte to AUX3/AUX4
  // bit0 = left joystick button  -> AUX3
  // bit1 = right joystick button -> AUX4 (can be used for BOXBEEPERON)
  espnow_rcData[AUX3] =     (MyData.switches & 0x01) ? 2000 : 1000;
  espnow_rcData[AUX4] =     (MyData.switches & 0x02) ? 2000 : 1000;
}

#endif
