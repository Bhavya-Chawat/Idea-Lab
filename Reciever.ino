#define BLYNK_TEMPLATE_ID "TMPL3nsysEAea"
#define BLYNK_TEMPLATE_NAME "smart roof cover"
#define BLYNK_AUTH_TOKEN "wMGwWtdMQJoKuMyoqxAEZLSvyh-kVNuj"

#include <WiFi.h>
#include <esp_now.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>

// Wi-Fi credentials
char ssid[] = "MARUTHI 4001";
char pass[] = "maruthi4001";

// Blynk virtual pins
#define BLYNK_MANUAL_VPIN V3
#define BLYNK_STATUS_VPIN V4

// Sensor data struct (from ESP-NOW sender)
typedef struct SensorData {
  bool isRain1;
  bool isRain2;
  bool isSunny;
} SensorData;

SensorData incomingData;

// Pins
const int topServoPin = 13;
const int bottomServoPin = 12;
const int in1 = 27;
const int in2 = 14;
const int in3 = 33;
const int in4 = 32;
const int manualButtonPin = 25;

// Servo objects
Servo topServo, bottomServo;

// States
bool manualOverride = false;
bool screenClosed = false;

// ========== Actuation Functions ==========
void unlockTop() {
  topServo.write(0);
  Serial.println("Top servo unlocked");
  delay(1000);
}
void lockTop() {
  topServo.write(90);
  Serial.println("Top servo locked");
  delay(1000);
}
void unlockBottom() {
  bottomServo.write(0);
  Serial.println("Bottom servo unlocked");
  delay(1000);
}
void lockBottom() {
  bottomServo.write(90);
  Serial.println("Bottom servo locked");
  delay(1000);
}

void rollDown() {
  Serial.println("Rolling screen down...");
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(5000);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  screenClosed = true;
  Serial.println("Screen closed");
}

void rollUp() {
  Serial.println("Rolling screen up...");
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(5000);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  screenClosed = false;
  Serial.println("Screen opened");
}

// ========== ESP-NOW Callback ==========
void onReceiveData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(SensorData));
  Serial.printf("Received → Rain1: %d, Rain2: %d, Sunny: %d\n",
                incomingData.isRain1, incomingData.isRain2, incomingData.isSunny);
}

// ========== Blynk Manual Override ==========
/*BLYNK_WRITE(BLYNK_MANUAL_VPIN) {
  Serial.println("BLYNK_WRITE(V3) triggered");
  manualOverride = param.asInt();
  if (manualOverride) {
    Serial.println("Manual override ON (from Blynk)");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Manual Override: ON");
    // unlockTop(); rollDown(); lockBottom();
  } else {
    Serial.println("Manual override OFF (from Blynk)");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Manual Override: OFF");
    // unlockBottom(); rollUp(); lockTop();
  }
}*/

void setup() {
  Serial.begin(115200);

  // Connect Wi-Fi for Blynk (comment if using only ESP-NOW)
  WiFi.begin(ssid, pass);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Connecting to Wi-Fi & Blynk...");

  // ESP-NOW setup
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onReceiveData);
  Serial.println("ESP-NOW ready");

  // Motor pins
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);

  // Servo setup
  topServo.setPeriodHertz(50);
  topServo.attach(topServoPin, 500, 2400);
  bottomServo.setPeriodHertz(50);
  bottomServo.attach(bottomServoPin, 500, 2400);

// Manual button
//  pinMode(manualButtonPin, INPUT_PULLUP);

// Initial Blynk sync
// Blynk.virtualWrite(BLYNK_MANUAL_VPIN, manualOverride);
//  Blynk.virtualWrite(BLYNK_STATUS_VPIN, "System Booted");
//  Serial.println("Receiver Ready");

  unlockBottom();
  lockTop();

}

void loop() {
  // Blynk.run();

  // ========== Manual Button Logic ==========
  /*
  static bool lastBtnState = HIGH;
  bool btnState = digitalRead(manualButtonPin);
  if (btnState == LOW && lastBtnState == HIGH) {
    delay(50);  // debounce
    manualOverride = !manualOverride;
    Blynk.virtualWrite(BLYNK_MANUAL_VPIN, manualOverride);

    if (manualOverride) {
      Serial.println("Manual override ON (Button)");
      Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Manual Override: ON (Button)");
      // unlockTop(); rollDown(); lockBottom();
    } else {
      Serial.println("Manual override OFF (Button)");
      Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Manual Override: OFF (Button)");
      // unlockBottom(); rollUp(); lockTop();
    }
  }
  lastBtnState = btnState;
  */

  // ========== Auto Mode ==========
  if (incomingData.isRain1 && incomingData.isRain2 && !screenClosed) {
    Serial.println("Auto: Both rain sensors triggered. Rolling down.");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Auto: Rain Detected");
    unlockTop(); rollDown(); lockBottom();
  } else if (incomingData.isSunny && !incomingData.isRain1 && !incomingData.isRain2 && screenClosed) {
    Serial.println("Auto: Sunny & dry. Rolling up.");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "Auto: Sunny, opening screen");
    unlockBottom(); rollUp(); lockTop();
  } else if ((incomingData.isRain1 || incomingData.isRain2) && !screenClosed) {
    Serial.println("Auto: Only one rain sensor triggered");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "One rain sensor triggered");
  } else {
    Serial.println("System OK");
    Blynk.virtualWrite(BLYNK_STATUS_VPIN, "System OK");
  }

  delay(100);
}
