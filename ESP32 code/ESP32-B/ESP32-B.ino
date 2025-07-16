/*
 * ESP32-B: GSM Data Transmitter (I²C Slave)
 * This ESP32 is connected to the SIM800L GSM module and receives GPS data
 * from ESP32-A via I²C, then transmits it to Firebase via cellular connection.
 * 
 * Hardware Connections:
 * - SIM800L GSM: VCC->3.7V, GND->GND, TX->GPIO20, RX->GPIO21, RST->GPIO6
 * - I²C from ESP32-A: SDA->GPIO8, SCL->GPIO9
 * 
 * Note: GPIO21 is shared between SIM800L TX and I²C SDA - this works because
 * I²C communication happens at different times than GSM communication
 */

#include <Wire.h>
#include <ArduinoJson.h>
#include <time.h>

// I²C configuration
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_SLAVE_ADDRESS 0x08

// SIM800L configuration
#define SIM800L_RST 6
HardwareSerial sim800l(1);

// Call trigger configuration
#define TRIGGER_PIN 2         // Pin that triggers the call when HIGH
bool lastPinState = LOW;      // Track previous pin state
bool callInProgress = false;  // Track if a call is currently being made
unsigned long firstTriggerTime = 0;  // Time of first trigger
bool firstTriggerDetected = false;   // Flag for first trigger
const unsigned long DOUBLE_TRIGGER_WINDOW = 500; // 500ms window for double trigger
const char* emergencyNumber = "+xxxyyyyyyyyy"; // Emergency contact number( where xxx is the country code and yyyyyyyyy is the emergency phone number)

// APN settings - replace with your carrier's APN
const char* apn = "internet";
const char* apn_user = "";
const char* apn_pass = "";

// Firebase configuration
const char* firebase_host = "gps-tracker-a713b-default-rtdb.firebaseio.com";//replace with your firebase host url
const char* firebase_auth = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";//replace with your firebase secret key

// Device identifier
String deviceId = "tracker_001";

// GPS data buffer
String receivedGPSData = "";
bool newGPSDataReceived = false;
unsigned long lastFirebaseUpdate = 0;
const unsigned long firebaseUpdateInterval = 10000; // Send to Firebase every 10 seconds

// System status
bool gsmReady = false;
bool internetConnected = false;

void setup() {
  Serial.begin(115200);
  
  Serial.println("ESP32-B: GSM Data Transmitter (I²C Slave)");
  Serial.println("=========================================");
  
  // Initialize I²C as slave
  Wire.begin(I2C_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  Wire.onReceive(onI2CReceive);
  Serial.printf("I²C slave initialized with address 0x%02X\n", I2C_SLAVE_ADDRESS);
  
  // Initialize SIM800L
  sim800l.begin(9600, SERIAL_8N1, 20, 21);
  
  // Configure trigger pin as input with internal pulldown
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  
  // Reset SIM800L
  pinMode(SIM800L_RST, OUTPUT);
  digitalWrite(SIM800L_RST, LOW);
  delay(1000);
  digitalWrite(SIM800L_RST, HIGH);
  delay(2000);
  
  // Initialize GSM connection
  if (initializeGSM()) {
    Serial.println("✓ GSM initialized successfully");
    gsmReady = true;
    
    if (connectToInternet()) {
      Serial.println("✓ Internet connection established");
      internetConnected = true;
    } else {
      Serial.println("❌ Failed to connect to internet");
    }
  } else {
    Serial.println("❌ GSM initialization failed");
  }
  
  Serial.println("Ready to receive GPS data from ESP32-A...");
  Serial.println("Emergency call: Double trigger pin 5 within 500ms");
}

void loop() {
  // Handle emergency call trigger
  handleEmergencyTrigger();
  
  // Process received GPS data
  if (newGPSDataReceived) {
    Serial.println("📡 New GPS data received from ESP32-A:");
    Serial.println(receivedGPSData);
    
    // Send to Firebase if connected
    if (internetConnected && millis() - lastFirebaseUpdate >= firebaseUpdateInterval) {
      sendToFirebase(receivedGPSData);
      lastFirebaseUpdate = millis();
    }
    
    newGPSDataReceived = false;
  }
  
  // Check GSM connection status periodically
  static unsigned long lastStatusCheck = 0;
  if (millis() - lastStatusCheck >= 30000) { // Check every 30 seconds
    checkGSMStatus();
    lastStatusCheck = millis();
  }
  
  delay(100);
}

void onI2CReceive(int numBytes) {
  receivedGPSData = "";
  
  // Read all available bytes
  while (Wire.available()) {
    char c = Wire.read();
    receivedGPSData += c;
  }
  
  if (receivedGPSData.length() > 0) {
    newGPSDataReceived = true;
    Serial.printf("📥 Received %d bytes via I²C\n", receivedGPSData.length());
  }
}

bool initializeGSM() {
  Serial.println("Initializing GSM module...");
  
  // Test AT command
  if (!sendATCommand("AT", "OK", 3000)) {
    Serial.println("❌ AT command failed");
    return false;
  }
  Serial.println("✓ AT command successful");
  
  // Disable echo
  sendATCommand("ATE0", "OK", 1000);
  
  // Check SIM card
  if (!sendATCommand("AT+CPIN?", "READY", 5000)) {
    Serial.println("❌ SIM card not ready");
    return false;
  }
  Serial.println("✓ SIM card ready");
  
  // Wait for network registration
  Serial.println("Waiting for network registration...");
  for (int i = 0; i < 30; i++) {
    if (sendATCommand("AT+CREG?", "+CREG: 0,1", 2000) || 
        sendATCommand("AT+CREG?", "+CREG: 0,5", 2000)) {
      Serial.println("✓ Network registered");
      return true;
    }
    delay(2000);
    Serial.print(".");
  }
  
  Serial.println("\n❌ Network registration timeout");
  return false;
}

bool connectToInternet() {
  Serial.println("Connecting to internet...");
  
  // Set connection type to GPRS
  if (!sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000)) {
    return false;
  }
  
  // Set APN
  String apnCmd = "AT+SAPBR=3,1,\"APN\",\"" + String(apn) + "\"";
  if (!sendATCommand(apnCmd.c_str(), "OK", 2000)) {
    return false;
  }
  
  // Set APN credentials if provided
  if (strlen(apn_user) > 0) {
    String userCmd = "AT+SAPBR=3,1,\"USER\",\"" + String(apn_user) + "\"";
    sendATCommand(userCmd.c_str(), "OK", 2000);
  }
  
  if (strlen(apn_pass) > 0) {
    String passCmd = "AT+SAPBR=3,1,\"PWD\",\"" + String(apn_pass) + "\"";
    sendATCommand(passCmd.c_str(), "OK", 2000);
  }
  
  // Open GPRS context
  if (!sendATCommand("AT+SAPBR=1,1", "OK", 10000)) {
    Serial.println("❌ Failed to open GPRS context");
    return false;
  }
  
  // Query GPRS context
  if (sendATCommand("AT+SAPBR=2,1", "OK", 5000)) {
    Serial.println("✓ Internet connection established");
    return true;
  }
  
  return false;
}

void sendToFirebase(String gpsData) {
  if (!internetConnected) {
    Serial.println("❌ No internet connection");
    return;
  }
  
  // Parse GPS data
  StaticJsonDocument<256> gpsDoc;
  DeserializationError error = deserializeJson(gpsDoc, gpsData);
  
  if (error) {
    Serial.println("❌ Failed to parse GPS data");
    return;
  }
  
  // Create Firebase payload
  StaticJsonDocument<512> firebaseDoc;
  firebaseDoc["deviceId"] = deviceId;
  firebaseDoc["latitude"] = gpsDoc["lat"];
  firebaseDoc["longitude"] = gpsDoc["lng"];
  firebaseDoc["altitude"] = gpsDoc["alt"];
  firebaseDoc["speed"] = gpsDoc["speed"];
  firebaseDoc["satellites"] = gpsDoc["sats"];
  firebaseDoc["hdop"] = gpsDoc["hdop"];
  firebaseDoc["age"] = gpsDoc["age"];
  firebaseDoc["timestamp"] = (int)time(nullptr);
  firebaseDoc["battery"] = random(20, 100); // Simulated battery level
  
  String payload;
  serializeJson(firebaseDoc, payload);
  
  // Send to Firebase
  if (sendHTTPRequest("PUT", "/trackers/" + deviceId + "/current.json", payload)) {
    Serial.println("✓ Data sent to Firebase successfully");
    Serial.printf("  Lat: %.6f, Lng: %.6f\n", (float)gpsDoc["lat"], (float)gpsDoc["lng"]);
  } else {
    Serial.println("❌ Failed to send data to Firebase");
  }
}

bool sendHTTPRequest(String method, String path, String payload) {
  // Initialize HTTP
  if (!sendATCommand("AT+HTTPINIT", "OK", 2000)) {
    Serial.println("❌ HTTP init failed");
    return false;
  }
  
  // Set HTTP parameters
  sendATCommand("AT+HTTPPARA=\"CID\",1", "OK", 2000);
  
  // Set URL
  String url = "https://" + String(firebase_host) + path + "?auth=" + String(firebase_auth);
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + url + "\"";
  if (!sendATCommand(urlCmd.c_str(), "OK", 2000)) {
    Serial.println("❌ HTTP URL failed");
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  
  // Set content type
  sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK", 2000);
  
  // Set data
  String dataCmd = "AT+HTTPDATA=" + String(payload.length()) + ",10000";
  if (sendATCommand(dataCmd.c_str(), "DOWNLOAD", 2000)) {
    sim800l.print(payload);
    delay(1000);
  }
  
  // Execute HTTP request (0=GET, 1=POST, 2=HEAD, 3=PUT)
  int actionType = (method == "PUT") ? 3 : 1;
  String actionCmd = "AT+HTTPACTION=" + String(actionType);
  if (!sendATCommand(actionCmd.c_str(), "OK", 2000)) {
    Serial.println("❌ HTTP action failed");
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  
  // Wait for response
  delay(5000);
  
  // Check response
  sim800l.println("AT+HTTPREAD");
  String response = readResponse(3000);
  
  // Clean up
  sendATCommand("AT+HTTPTERM", "OK", 2000);
  
  return response.indexOf("200") > -1;
}

void checkGSMStatus() {
  // Check signal strength
  sim800l.println("AT+CSQ");
  String response = readResponse(2000);
  
  if (response.indexOf("+CSQ:") > -1) {
    int rssi_start = response.indexOf("+CSQ: ") + 6;
    int rssi_end = response.indexOf(",", rssi_start);
    if (rssi_end > rssi_start) {
      int rssi = response.substring(rssi_start, rssi_end).toInt();
      Serial.printf("📶 Signal strength: %d", rssi);
      
      if (rssi >= 15) Serial.println(" (Good)");
      else if (rssi >= 10) Serial.println(" (Fair)");
      else if (rssi >= 5) Serial.println(" (Poor)");
      else Serial.println(" (Very Poor)");
    }
  }
}

bool sendATCommand(const char* command, const char* expected, unsigned long timeout) {
  sim800l.println(command);
  String response = readResponse(timeout);
  return response.indexOf(expected) > -1;
}

String readResponse(unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  
  while (millis() - start < timeout) {
    if (sim800l.available()) {
      char c = sim800l.read();
      response += c;
    }
    delay(1);
  }
  
  return response;
}

void handleEmergencyTrigger() {
  // Read current pin state
  bool currentPinState = digitalRead(TRIGGER_PIN);
  
  // Check for rising edge (LOW to HIGH transition)
  if (currentPinState == HIGH && lastPinState == LOW && !callInProgress) {
    unsigned long currentTime = millis();
    
    if (!firstTriggerDetected) {
      // First trigger detected
      firstTriggerDetected = true;
      firstTriggerTime = currentTime;
      Serial.println("🚨 First trigger detected. Waiting for second trigger...");
    } else {
      // Second trigger detected - check if within time window
      if (currentTime - firstTriggerTime <= DOUBLE_TRIGGER_WINDOW) {
        Serial.println("🚨 Double trigger detected - Making emergency call...");
        makeEmergencyCall();
        firstTriggerDetected = false; // Reset for next double trigger
      } else {
        // Too late - treat as new first trigger
        firstTriggerDetected = true;
        firstTriggerTime = currentTime;
        Serial.println("⏰ Previous trigger expired. New first trigger detected.");
      }
    }
  }
  
  // Reset first trigger if window expires
  if (firstTriggerDetected && (millis() - firstTriggerTime > DOUBLE_TRIGGER_WINDOW)) {
    firstTriggerDetected = false;
    Serial.println("⏰ First trigger expired. Ready for new double trigger.");
  }
  
  // Update previous pin state
  lastPinState = currentPinState;
}

void makeEmergencyCall() {
  if (!gsmReady) {
    Serial.println("❌ GSM not ready - cannot make call");
    return;
  }
  
  callInProgress = true;
  
  Serial.println("📞 Initiating emergency call...");
  String dialCmd = "ATD" + String(emergencyNumber) + ";";
  sim800l.println(dialCmd);
  
  // Monitor call status
  delay(2000);
  String response = readResponse(3000);
  Serial.println("Call response: " + response);
  
  // Wait for call to establish
  Serial.println("📞 Call in progress...");
  delay(20000); // Wait for 20 seconds
  
  // Hang up call
  Serial.println("📞 Hanging up call...");
  sim800l.println("ATH");
  delay(1000);
  
  callInProgress = false;
  Serial.println("✓ Emergency call completed. Ready for next double trigger.");
}
