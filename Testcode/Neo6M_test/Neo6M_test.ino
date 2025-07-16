#include <TinyGPS++.h>

TinyGPSPlus gps;
float latitude, longitude;

void setup() {
  Serial1.begin(9600, SERIAL_8N1, 20, 21); // RX=20, TX=21
  Serial.begin(9600);
  Serial.println("ESP32 SuperMini GPS Module Test (Hardware Serial)");
  delay(1000);
}

void loop() {
  while (Serial1.available()) {
    int data = Serial1.read();
    if (gps.encode(data)) {
      if (gps.location.isValid()) {
        latitude = (gps.location.lat());
        longitude = (gps.location.lng());
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("Altitude: ");
        Serial.println(gps.altitude.meters());
        Serial.println("---");
        delay(1000);
      } else {
        Serial.println("Waiting for GPS fix...");
        delay(500);
      }
    }
  }
}