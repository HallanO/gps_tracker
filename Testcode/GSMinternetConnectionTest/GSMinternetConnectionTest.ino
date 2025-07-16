// Pin definitions for ESP32-C3 Super Mini
#define SIM800L_RST 6  // Connect to SIM800L RST (optional)

// Use Hardware Serial1 for SIM800L communication
// ESP32-C3 Serial1: TX=GPIO21, RX=GPIO20 (default pins)
// Connect SIM800L RX to GPIO21, SIM800L TX to GPIO20
HardwareSerial sim800l(1);

// APN settings - replace with your carrier's APN
const char* apn = "internet";  // Common APN, change as needed
const char* apn_user = "";     // Usually empty
const char* apn_pass = "";     // Usually empty

// Test server for connectivity check
const char* test_server = "httpbin.org";
const int test_port = 80;

void setup() {
  Serial.begin(115200);
  
  // Initialize hardware serial for SIM800L
  // Serial1 pins: TX=GPIO21, RX=GPIO20
  sim800l.begin(9600, SERIAL_8N1, 20, 21);  // baud, config, RX pin, TX pin
  
  // Optional: Reset pin setup
  if (SIM800L_RST != -1) {
    pinMode(SIM800L_RST, OUTPUT);
    digitalWrite(SIM800L_RST, LOW);
    delay(1000);
    digitalWrite(SIM800L_RST, HIGH);
    delay(2000);
  }
  
  Serial.println("ESP32-C3 + SIM800L Internet Connection Test");
  Serial.println("==========================================");
  
  // Initialize SIM800L
  if (initializeSIM800L()) {
    Serial.println("SIM800L initialized successfully!");
    
    // Test internet connection
    if (testInternetConnection()) {
      Serial.println("\n✓ Internet connection test PASSED!");
      Serial.println("Your SIM800L is connected to the internet.");
    } else {
      Serial.println("\n✗ Internet connection test FAILED!");
      Serial.println("Check your SIM card, signal strength, and APN settings.");
    }
  } else {
    Serial.println("✗ Failed to initialize SIM800L!");
  }
}

void loop() {
  // Keep monitoring signal strength
  delay(30000);  // Check every 30 seconds
  checkSignalStrength();
}

bool initializeSIM800L() {
  Serial.println("Initializing SIM800L...");
  
  // Wait for module to be ready
  delay(3000);
  
  // Test AT command
  if (!sendATCommand("AT", "OK", 2000)) {
    Serial.println("✗ AT command failed");
    return false;
  }
  Serial.println("✓ AT command successful");
  
  // Disable echo
  sendATCommand("ATE0", "OK", 1000);
  
  // Check SIM card status
  if (!sendATCommand("AT+CPIN?", "READY", 5000)) {
    Serial.println("✗ SIM card not ready");
    return false;
  }
  Serial.println("✓ SIM card ready");
  
  // Wait for network registration
  Serial.println("Waiting for network registration...");
  for (int i = 0; i < 30; i++) {
    if (sendATCommand("AT+CREG?", "+CREG: 0,1", 2000) || 
        sendATCommand("AT+CREG?", "+CREG: 0,5", 2000)) {
      Serial.println("✓ Network registered");
      break;
    }
    delay(2000);
    if (i == 29) {
      Serial.println("✗ Network registration timeout");
      return false;
    }
  }
  
  // Check signal strength
  checkSignalStrength();
  
  // Configure GPRS
  if (!configureGPRS()) {
    Serial.println("✗ GPRS configuration failed");
    return false;
  }
  
  return true;
}

bool configureGPRS() {
  Serial.println("Configuring GPRS...");
  
  // Set connection type to GPRS
  if (!sendATCommand("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000)) {
    return false;
  }
  
  // Set APN
  String apnCmd = "AT+SAPBR=3,1,\"APN\",\"" + String(apn) + "\"";
  if (!sendATCommand(apnCmd.c_str(), "OK", 2000)) {
    return false;
  }
  
  // Set APN user if provided
  if (strlen(apn_user) > 0) {
    String userCmd = "AT+SAPBR=3,1,\"USER\",\"" + String(apn_user) + "\"";
    sendATCommand(userCmd.c_str(), "OK", 2000);
  }
  
  // Set APN password if provided
  if (strlen(apn_pass) > 0) {
    String passCmd = "AT+SAPBR=3,1,\"PWD\",\"" + String(apn_pass) + "\"";
    sendATCommand(passCmd.c_str(), "OK", 2000);
  }
  
  // Open GPRS context
  if (!sendATCommand("AT+SAPBR=1,1", "OK", 10000)) {
    Serial.println("✗ Failed to open GPRS context");
    return false;
  }
  
  // Query GPRS context
  if (sendATCommand("AT+SAPBR=2,1", "OK", 5000)) {
    Serial.println("✓ GPRS context established");
    return true;
  }
  
  return false;
}

bool testInternetConnection() {
  Serial.println("Testing internet connection...");
  
  // Initialize HTTP service
  if (!sendATCommand("AT+HTTPINIT", "OK", 2000)) {
    Serial.println("✗ HTTP init failed");
    return false;
  }
  
  // Set HTTP parameters
  if (!sendATCommand("AT+HTTPPARA=\"CID\",1", "OK", 2000)) {
    Serial.println("✗ HTTP CID parameter failed");
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  
  // Set URL for test
  String urlCmd = "AT+HTTPPARA=\"URL\",\"http://" + String(test_server) + "/ip\"";
  if (!sendATCommand(urlCmd.c_str(), "OK", 2000)) {
    Serial.println("✗ HTTP URL parameter failed");
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  
  // Perform HTTP GET request
  if (!sendATCommand("AT+HTTPACTION=0", "OK", 2000)) {
    Serial.println("✗ HTTP action failed");
    sendATCommand("AT+HTTPTERM", "OK", 2000);
    return false;
  }
  
  // Wait for HTTP response
  delay(5000);
  
  // Check if we got a response
  sim800l.println("AT+HTTPREAD");
  String response = readResponse(5000);
  
  // Clean up
  sendATCommand("AT+HTTPTERM", "OK", 2000);
  
  if (response.indexOf("200") > -1 || response.indexOf("{") > -1) {
    Serial.println("✓ HTTP request successful");
    Serial.println("Response preview:");
    Serial.println(response.substring(0, 200));
    return true;
  } else {
    Serial.println("✗ HTTP request failed");
    Serial.println("Response: " + response);
    return false;
  }
}

void checkSignalStrength() {
  sim800l.println("AT+CSQ");
  String response = readResponse(2000);
  
  if (response.indexOf("+CSQ:") > -1) {
    int rssi_start = response.indexOf("+CSQ: ") + 6;
    int rssi_end = response.indexOf(",", rssi_start);
    if (rssi_end > rssi_start) {
      int rssi = response.substring(rssi_start, rssi_end).toInt();
      Serial.print("Signal strength: ");
      Serial.print(rssi);
      Serial.print(" (");
      
      if (rssi == 0) Serial.print("≤ -115 dBm - No signal");
      else if (rssi == 1) Serial.print("-111 dBm - Marginal");
      else if (rssi >= 2 && rssi <= 9) Serial.print("Poor");
      else if (rssi >= 10 && rssi <= 14) Serial.print("Fair");
      else if (rssi >= 15 && rssi <= 19) Serial.print("Good");
      else if (rssi >= 20 && rssi <= 30) Serial.print("Excellent");
      else if (rssi == 31) Serial.print("≥ -51 dBm - Maximum");
      else Serial.print("Unknown");
      
      Serial.println(")");
    }
  }
}

bool sendATCommand(const char* command, const char* expected, unsigned long timeout) {
  sim800l.println(command);
  return waitForResponse(expected, timeout);
}

bool waitForResponse(const char* expected, unsigned long timeout) {
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
      Serial.print(c);  // Echo to serial monitor
    }
    delay(1);
  }
  
  return response;
}