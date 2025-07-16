/*
 * ESP32-A: GPS Data Collector (I²C Master)
 * This ESP32 is connected to the Neo-6M GPS module and sends GPS data
 * to ESP32-B via I²C communication protocol.
 * 
 * Hardware Connections:
 * - Neo-6M GPS: VCC->3.3V, GND->GND, TX->GPIO20, RX->GPIO21
 * - I²C to ESP32-B: SDA->GPIO21, SCL->GPIO22
 * 
 * Note: GPIO21 is shared between GPS RX and I²C SDA, but this works
 * because I²C is bidirectional and GPS only sends data to ESP32
 */

#include <Wire.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

// I²C configuration
#define I2C_SDA 8
#define I2C_SCL 9
#define ESP32_B_ADDRESS 0x08  // I²C address of ESP32-B

// GPS setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use Serial1 for GPS

// GPS data structure
struct GPSData {
  float latitude;
  float longitude;
  float altitude;
  float speed;
  uint8_t satellites;
  uint32_t timestamp;
  uint16_t hdop;
  uint32_t age;
  bool isValid;
};

GPSData currentGPS;
unsigned long lastGPSCheck = 0;
unsigned long lastI2CTransmit = 0;
const unsigned long gpsCheckInterval = 1000;    // Check GPS every second
const unsigned long i2cTransmitInterval = 5000; // Send to ESP32-B every 5 seconds

void setup() {
  Serial.begin(115200);
  
  // Initialize I²C as master
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("ESP32-A: GPS Data Collector (I²C Master)");
  Serial.println("========================================");
  
  // Initialize GPS serial communication
  gpsSerial.begin(9600, SERIAL_8N1, 20, 21); // RX=20, TX=21
  Serial.println("GPS Serial initialized on pins RX=20, TX=21");
  
  // Initialize GPS data structure
  currentGPS.isValid = false;
  
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Continuously read GPS data
  readGPSData();
  
  // Send GPS data to ESP32-B at specified intervals
  if (millis() - lastI2CTransmit >= i2cTransmitInterval) {
    if (currentGPS.isValid) {
      sendGPSDataToESP32B();
      Serial.println("✓ GPS data sent to ESP32-B");
    } else {
      Serial.println("⚠ No valid GPS data to send");
    }
    lastI2CTransmit = millis();
  }
  
  delay(100);
}

void readGPSData() {
  while (gpsSerial.available()) {
    int data = gpsSerial.read();
    if (gps.encode(data)) {
      if (gps.location.isValid()) {
        // Update GPS data structure
        currentGPS.latitude = gps.location.lat();
        currentGPS.longitude = gps.location.lng();
        currentGPS.altitude = gps.altitude.meters();
        currentGPS.speed = gps.speed.kmph();
        currentGPS.satellites = gps.satellites.value();
        currentGPS.timestamp = millis();
        currentGPS.hdop = gps.hdop.value();
        currentGPS.age = gps.location.age();
        currentGPS.isValid = true;
        
        // Print GPS data every second to avoid spam
        if (millis() - lastGPSCheck >= gpsCheckInterval) {
          printGPSData();
          lastGPSCheck = millis();
        }
      } else {
        currentGPS.isValid = false;
        if (millis() - lastGPSCheck >= gpsCheckInterval) {
          Serial.println("⏳ Waiting for GPS fix...");
          lastGPSCheck = millis();
        }
      }
    }
  }
}

void printGPSData() {
  Serial.println("📍 GPS Data:");
  Serial.printf("  Latitude: %.6f\n", currentGPS.latitude);
  Serial.printf("  Longitude: %.6f\n", currentGPS.longitude);
  Serial.printf("  Altitude: %.2f m\n", currentGPS.altitude);
  Serial.printf("  Speed: %.2f km/h\n", currentGPS.speed);
  Serial.printf("  Satellites: %d\n", currentGPS.satellites);
  Serial.printf("  HDOP: %d\n", currentGPS.hdop);
  Serial.printf("  Age: %d ms\n", currentGPS.age);
  Serial.println();
}

void sendGPSDataToESP32B() {
  // Create JSON string with GPS data
  StaticJsonDocument<256> doc;
  doc["lat"] = currentGPS.latitude;
  doc["lng"] = currentGPS.longitude;
  doc["alt"] = currentGPS.altitude;
  doc["speed"] = currentGPS.speed;
  doc["sats"] = currentGPS.satellites;
  doc["hdop"] = currentGPS.hdop;
  doc["age"] = currentGPS.age;
  doc["timestamp"] = currentGPS.timestamp;
  doc["valid"] = currentGPS.isValid;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send JSON string over I²C
  Wire.beginTransmission(ESP32_B_ADDRESS);
  Wire.write((const uint8_t*)jsonString.c_str(), jsonString.length());
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("📤 I²C transmission successful");
  } else {
    Serial.printf("❌ I²C transmission failed with error: %d\n", error);
    printI2CError(error);
  }
}

void printI2CError(byte error) {
  switch(error) {
    case 1:
      Serial.println("   Data too long to fit in transmit buffer");
      break;
    case 2:
      Serial.println("   Received NACK on transmit of address");
      break;
    case 3:
      Serial.println("   Received NACK on transmit of data");
      break;
    case 4:
      Serial.println("   Other error");
      break;
    case 5:
      Serial.println("   Timeout");
      break;
    default:
      Serial.println("   Unknown error");
      break;
  }
}