// ESP32 SIM800L Call Example with Pin 5 Trigger
// Use hardware serial instead of SoftwareSerial
// Define which hardware serial to use (ESP32 has multiple)
#define SIM800_SERIAL Serial1 // Using Serial1 (GPIO16=RX, GPIO17=TX by default)
#define TRIGGER_PIN 5         // Pin that triggers the call when HIGH

bool lastPinState = LOW;      // Track previous pin state
bool callInProgress = false;  // Track if a call is currently being made
unsigned long firstTriggerTime = 0;  // Time of first trigger
bool firstTriggerDetected = false;   // Flag for first trigger
const unsigned long DOUBLE_TRIGGER_WINDOW = 500; // 500ms window for double trigger

void setup() {
  // Begin serial communication with Arduino IDE (Serial Monitor)
  Serial.begin(9600);
  
  // Begin serial communication with SIM800L using hardware serial
  // ESP32 Serial1 default pins: RX=GPIO16, TX=GPIO17
  SIM800_SERIAL.begin(9600, SERIAL_8N1, 20, 21); // (baud, config, RX pin, TX pin)
  
  // Configure trigger pin as input with internal pulldown
  pinMode(TRIGGER_PIN, INPUT_PULLDOWN);
  
  Serial.println("Initializing SIM800L...");
  delay(1000);
  
  // Test SIM800L connection
  SIM800_SERIAL.println("AT");
  updateSerial();
  delay(1000);
  
  Serial.println("Ready. Waiting for double trigger on pin 5 (within 500ms)...");
}

void loop() {
  // Read current pin state
  bool currentPinState = digitalRead(TRIGGER_PIN);
  
  // Check for rising edge (LOW to HIGH transition)
  if (currentPinState == HIGH && lastPinState == LOW && !callInProgress) {
    unsigned long currentTime = millis();
    
    if (!firstTriggerDetected) {
      // First trigger detected
      firstTriggerDetected = true;
      firstTriggerTime = currentTime;
      Serial.println("First trigger detected. Waiting for second trigger...");
    } else {
      // Second trigger detected - check if within time window
      if (currentTime - firstTriggerTime <= DOUBLE_TRIGGER_WINDOW) {
        Serial.println("Double trigger detected - Making call...");
        makeCall();
        firstTriggerDetected = false; // Reset for next double trigger
      } else {
        // Too late - treat as new first trigger
        firstTriggerDetected = true;
        firstTriggerTime = currentTime;
        Serial.println("Previous trigger expired. New first trigger detected.");
      }
    }
  }
  
  // Reset first trigger if window expires
  if (firstTriggerDetected && (millis() - firstTriggerTime > DOUBLE_TRIGGER_WINDOW)) {
    firstTriggerDetected = false;
    Serial.println("First trigger expired. Ready for new double trigger.");
  }
  
  // Update previous pin state
  lastPinState = currentPinState;
  
  // Handle serial communication
  updateSerial();
  
  // Small delay to prevent excessive polling
  delay(50);
}

void makeCall() {
  callInProgress = true;
  
  Serial.println("Initiating call...");
  SIM800_SERIAL.println("ATD+256751505979;"); // Change with your phone number
  updateSerial();
  
  // Wait for call to establish
  delay(20000); // Wait for 20 seconds
  
  Serial.println("Hanging up call...");
  SIM800_SERIAL.println("ATH"); // Hang up
  updateSerial();
  
  callInProgress = false;
  Serial.println("Call completed. Ready for next double trigger.");
}

void updateSerial() {
  delay(50);
  while (Serial.available()) {
    SIM800_SERIAL.write(Serial.read()); // Forward from Serial Monitor to SIM800L
  }
  while (SIM800_SERIAL.available()) {
    Serial.write(SIM800_SERIAL.read()); // Forward from SIM800L to Serial Monitor
  }
}