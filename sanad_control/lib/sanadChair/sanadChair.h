#ifndef SANAD_CHAIR_H
#define SANAD_CHAIR_H

#include "l298n.h"
#include "ultrasonic.h"
#include "RosRobotNode.h"

class SanadChair {
public:
  // Constructor
  SanadChair();
  
  // Destructor
  ~SanadChair();
  
  // Initialization methods
  bool begin(const char* wifiSsid, const char* wifiPassword,
             const char* agentIp, uint16_t agentPort);
  
  // Main update method - call this in loop()
  void update();
  
  // Access to individual components (if needed)
  L298N* getMotorController();
  UltrasonicSensor* getSensor();
  RosRobotNode* getRosNode();

private:
  L298N* _l298n;
  UltrasonicSensor* _ultrasonic;
  RosRobotNode* _rosNode;
  
  bool _initialized;
};

#endif