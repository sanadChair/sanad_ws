#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>
#include "config.h"

class UltrasonicSensor {
public:
  void init();
  long getDistance();

private:
  const uint8_t trig_pin = TRIG_PIN;
  const uint8_t echo_pin = ECHO_PIN;
  static const long TIMEOUT = 50000; // 50ms timeout
};

#endif