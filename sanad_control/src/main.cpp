#include <Arduino.h>
#include "sanadChair.h"
#include "config.h"


SanadChair* chair = nullptr;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // Create Sanad Chair instance with pin configuration
  chair = new SanadChair();
  
  // Initialize the chair with WiFi and ROS settings
  if (!chair->begin(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT)) {
    Serial.println("FATAL ERROR: Failed to initialize Sanad Chair!");
    while(1) {
      delay(1000); // Halt execution
    }
  }
}


void loop() {
  // Update the chair - this handles all ROS communication
  chair->update();
}