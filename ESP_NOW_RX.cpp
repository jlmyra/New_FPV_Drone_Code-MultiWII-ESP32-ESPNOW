
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
typedef struct struct_message {
  uint8_t throttle;
  uint8_t yaw;
  uint8_t pitch;
  uint8_t roll;
  uint8_t AUX1;
  uint8_t AUX2;
  uint8_t switches;
} struct_message;

struct_message MyData;

// Structure to send acknowledgment data back to transmitter
typedef struct struct_ack {
  float lat;
  float lon;
  int16_t heading;
  int16_t pitch;
  int16_t roll;
  int32_t alt;
  uint8_t flags;
} struct_ack;

struct_ack espnowAckPayload;

static unsigned long lastRecvTime = 0;
static bool dataReceived = false;

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
  espnowAckPayload.lat = 0;
  espnowAckPayload.lon = 0;
  espnowAckPayload.heading = 0;
  espnowAckPayload.pitch = 0;
  espnowAckPayload.roll = 0;
  espnowAckPayload.alt = 0;
  espnowAckPayload.flags = 0;
}

// Callback function executed when data is received
// ESP32 Arduino Core 2.x uses esp_now_recv_info structure
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&MyData, incomingData, sizeof(MyData));
  dataReceived = true;
  lastRecvTime = millis();
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

  // Check if data was lost (no signal for more than 1 second)
  if (now - lastRecvTime > 1000) {
    // Signal lost - reset to safe values
    resetESPNOWData();
  }

  // Prepare acknowledgment payload with current drone status
  espnowAckPayload.lat = 35.62;
  espnowAckPayload.lon = 139.68;
  espnowAckPayload.heading = att.heading;
  espnowAckPayload.pitch = att.angle[PITCH];
  espnowAckPayload.roll = att.angle[ROLL];
  espnowAckPayload.alt = alt.EstAlt;
  memcpy(&espnowAckPayload.flags, &f, 1); // first byte of status flags

  // Map received data to RC channels
  // If your channels are inverted, reverse the map values
  espnow_rcData[THROTTLE] = map(MyData.throttle, 0, 255, 2000, 1000);
  espnow_rcData[ROLL] =     map(MyData.yaw,      0, 255, 2000, 1000);
  espnow_rcData[PITCH] =    map(MyData.pitch,    0, 255, 1000, 2000);
  espnow_rcData[YAW] =      map(MyData.roll,     0, 255, 2000, 1000);

  espnow_rcData[AUX1] =     map(MyData.AUX1,     0, 1, 2000, 1000);
  espnow_rcData[AUX2] =     map(MyData.AUX2,     0, 1, 2000, 1000);
}

#endif
