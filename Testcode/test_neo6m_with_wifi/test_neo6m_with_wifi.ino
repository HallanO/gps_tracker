#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>
#include <time.h>
#include <TinyGPS++.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// WiFi credentials
const char* ssid = "Lte4";
const char* password = "123456789";

// Firebase configuration
#define FIREBASE_HOST "gps-tracker-a713b-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "XkvlEARvrpEYPPWX8aVd5CFmYz2M0fvjYsHLz9pC"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// GPS setup
TinyGPSPlus gps;
float latitude, longitude;
bool gpsDataValid = false;

// Device identifier
String deviceId = "tracker_001";

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 10000; // 10 seconds
unsigned long lastGPSCheck = 0;
const unsigned long gpsCheckInterval = 1000; // Check GPS every second

void setup() {
  Serial.begin(115200);
  
  // Initialize GPS serial communication
  Serial1.begin(9600, SERIAL_8N1, 20, 21); // RX=20, TX=21
  Serial.println("ESP32 GPS Tracker with NEO-6M");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to WiFi. IP: ");
  Serial.println(WiFi.localIP());
  
  // Configure Firebase
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  
  // Initialize firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Serial.println("Firebase initialized");
  
  // Initialize time
  configTime(3 * 3600, 0, "pool.ntp.org"); // UTC+3 for East Africa
  
  Serial.println("Waiting for GPS fix...");
}

void loop() {
  // Read GPS data continuously
  readGPSData();
  
  // Send location to Firebase at specified intervals
  if (millis() - lastUpdate >= updateInterval) {
    if (gpsDataValid) {
      sendLocationToFirebase();
      lastUpdate = millis();
    } else {
      Serial.println("No valid GPS data available. Skipping Firebase update.");
      lastUpdate = millis(); // Still update the timer to avoid spam
    }
  }
  
  delay(100);
}

void readGPSData() {
  while (Serial1.available()) {
    int data = Serial1.read();
    if (gps.encode(data)) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        gpsDataValid = true;
        
        // Print GPS data every second to avoid spam
        if (millis() - lastGPSCheck >= gpsCheckInterval) {
          Serial.print("GPS Fix - Lat: ");
          Serial.print(latitude, 6);
          Serial.print(", Lng: ");
          Serial.print(longitude, 6);
          Serial.print(", Satellites: ");
          Serial.print(gps.satellites.value());
          Serial.print(", Altitude: ");
          Serial.print(gps.altitude.meters());
          Serial.println(" m");
          lastGPSCheck = millis();
        }
      } else {
        gpsDataValid = false;
        if (millis() - lastGPSCheck >= gpsCheckInterval) {
          Serial.println("Waiting for GPS fix...");
          lastGPSCheck = millis();
        }
      }
    }
  }
}

void sendLocationToFirebase() {
  if (WiFi.status() == WL_CONNECTED && gpsDataValid) {
    // Create timestamp
    time_t now;
    time(&now);
    
    // Create JSON object with real GPS data
    FirebaseJson json;
    json.set("deviceId", deviceId);
    json.set("latitude", latitude);
    json.set("longitude", longitude);
    json.set("timestamp", (int)now);
    json.set("satellites", gps.satellites.value());
    json.set("altitude", gps.altitude.meters());
    json.set("speed", gps.speed.kmph()); // Real speed from GPS
    json.set("battery", random(20, 100)); // Still simulated - add real battery monitoring if needed
    
    // Add GPS quality indicators
    json.set("hdop", gps.hdop.value());
    json.set("age", gps.location.age());
    
    // Path for current location
    String currentPath = "/trackers/" + deviceId + "/current";
    
    // Path for location history
    String historyPath = "/trackers/" + deviceId + "/history/" + String((int)now);
    
    // Send current location
    if (Firebase.RTDB.setJSON(&fbdo, currentPath.c_str(), &json)) {
      Serial.println("✓ Current location sent successfully");
      Serial.printf("  Lat: %.6f, Lng: %.6f\n", latitude, longitude);
    } else {
      Serial.println("✗ Failed to send current location");
      Serial.println(fbdo.errorReason());
    }
    
    // Send to history
    if (Firebase.RTDB.setJSON(&fbdo, historyPath.c_str(), &json)) {
      Serial.println("✓ Location history updated");
    } else {
      Serial.println("✗ Failed to update history");
      Serial.println(fbdo.errorReason());
    }
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected");
    }
    if (!gpsDataValid) {
      Serial.println("No valid GPS data");
    }
  }
}