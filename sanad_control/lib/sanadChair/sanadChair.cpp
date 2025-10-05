#include "sanadChair.h"

SanadChair::SanadChair()
  : _initialized(false) {
  
  // Create component instances
  _l298n = new L298N();
  _ultrasonic = new UltrasonicSensor();
  _rosNode = new RosRobotNode(_l298n, _ultrasonic);
}

SanadChair::~SanadChair() {
  // Clean up dynamically allocated objects
  delete _rosNode;
  delete _ultrasonic;
  delete _l298n;
}

bool SanadChair::begin(const char* wifiSsid, const char* wifiPassword,
                       const char* agentIp, uint16_t agentPort) {
  
  Serial.println("========================================");
  Serial.println("      Initializing Sanad Chair");
  Serial.println("========================================");
  
  // Initialize hardware components
  Serial.println("Initializing motor controller...");
  _l298n->initialMotors();
  
  Serial.println("Initializing ultrasonic sensor...");
  _ultrasonic->init();
  
  // Initialize WiFi
  Serial.println("Initializing WiFi connection...");
  if (!_rosNode->initWiFi(wifiSsid, wifiPassword, agentIp, agentPort)) {
    Serial.println("ERROR: WiFi initialization failed!");
    return false;
  }
  
  // Initialize ROS
  Serial.println("Initializing ROS2 node...");
  if (!_rosNode->initROS()) {
    Serial.println("ERROR: ROS2 initialization failed!");
    return false;
  }
  
  _initialized = true;
  Serial.println("========================================");
  Serial.println("   Sanad Chair Ready!");
  Serial.println("========================================");
  
  return true;
}

void SanadChair::update() {
  if (!_initialized) {
    Serial.println("WARNING: SanadChair not initialized! Call begin() first.");
    return;
  }
  
  // Spin ROS executor to process callbacks
  _rosNode->spin();
}

// Getter methods for component access
L298N* SanadChair::getMotorController() {
  return _l298n;
}

UltrasonicSensor* SanadChair::getSensor() {
  return _ultrasonic;
}

RosRobotNode* SanadChair::getRosNode() {
  return _rosNode;
}