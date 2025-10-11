#include "ultrasonic.h"


void UltrasonicSensor::init() {
  pinMode(this->echo_pin, OUTPUT);
  pinMode(this->trig_pin, INPUT);
}

long UltrasonicSensor::getDistance() {
  // Trigger the sensor
  digitalWrite(this->trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trig_pin, LOW);

  // Read the echo pulse with a specific timeout
  long duration = pulseIn(this->echo_pin, HIGH, TIMEOUT);

  // If duration is 0, no object detected or sensor disconnected
  if (duration == 0) {
    return 0;
  }

  // Calculate distance in centimeters
  return duration * 0.0343 / 2;
}