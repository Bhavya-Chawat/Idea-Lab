#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Receiver ESP32 MAC address
uint8_t receiverAddress[] = { 0x68, 0x25, 0xDD, 0xFD, 0xA9, 0x80 };

// Sensor pins
const int rainSensor1Pin = 32;
const int rainSensor2Pin = 33;
const int ldrPin = 34;

// Brightness threshold for LDR (adjustable)
const int ldrThreshold = 200;

// Data structure to send
typedef struct {
  bool isRain1;
  bool isRain2;
  bool isSunny;
} SensorData;

SensorData sensorData;

// Callback function for send status
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);

  // Configure sensor pins
  pinMode(rainSensor1Pin, INPUT);
  pinMode(rainSensor2Pin, INPUT);
  pinMode(ldrPin, INPUT);

  // Initialize Wi-Fi in station mode and fix channel to 1
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed!");
    return;
  }

  esp_now_register_send_cb(onDataSent);

  // Register receiver ESP32 as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(receiverAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
  }

  Serial.println("ESP-NOW Sender Ready (Channel 1)");
}

void loop() {
  // Read sensors and populate data structure
  sensorData.isRain1 = (digitalRead(rainSensor1Pin) == LOW); // True if rain
  sensorData.isRain2 = (digitalRead(rainSensor2Pin) == LOW);
  sensorData.isSunny = (digitalRead(ldrPin) == LOW);         // True if bright

  // Print current sensor states
  Serial.print("Rain1: ");
  Serial.print(sensorData.isRain1);
  Serial.print(" | Rain2: ");
  Serial.print(sensorData.isRain2);
  Serial.print(" | Sunny: ");
  Serial.println(sensorData.isSunny);

  // Send sensor data via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.print("Send failed: ");
    Serial.println(result);
  }

  delay(5000); // Delay between transmissions
}
