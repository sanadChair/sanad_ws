#include "l298n.h"
#include <Arduino.h>
#include "config.h"

void L298N::initialMotors(){
    // Configure pins as outputs
    for (int i = 0; i < 6; i++){
        pinMode(this->driverPins[i], OUTPUT);
    }
    analogWrite(this->driverPins[4], this->speed);
	analogWrite(this->driverPins[5], this->speed);
    // stop the motors
    L298N::stop();
}

void L298N::moveForward() {
    Serial.println("moving Forward");
    // Motor A forward
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    // Motor B forward
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4

    this->stateMotorA = 1;
    this->stateMotorB = 1;
//     analogWrite(this->driverPins[4], this->speed);
// 	analogWrite(this->driverPins[5], this->speed);
}

void L298N::moveBackward() {
    Serial.println("moving Backward");
    // Motor A backward
    digitalWrite(this->driverPins[0], HIGH); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    
    // Motor B forward
    digitalWrite(this->driverPins[2], HIGH); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4

    // analogWrite(this->driverPins[4], this->speed);
	// analogWrite(this->driverPins[5], this->speed);
}

void L298N::rotateLeft() {
    Serial.println("moving Left");
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], LOW); // IN1
    digitalWrite(this->driverPins[1], HIGH); // IN2
    digitalWrite(this->driverPins[2], HIGH); // IN3
    digitalWrite(this->driverPins[3], LOW); // IN4

    // analogWrite(this->driverPins[4], this->speed);
	// analogWrite(this->driverPins[5], this->speed);
}

void L298N::rotateRight() {
    Serial.println("moving Right");
    // Rotate in place by stopping one motor
    digitalWrite(this->driverPins[0], HIGH); // IN1
    digitalWrite(this->driverPins[1], LOW); // IN2
    digitalWrite(this->driverPins[2], LOW); // IN3
    digitalWrite(this->driverPins[3], HIGH); // IN4

    // analogWrite(this->driverPins[4], this->speed);
	// analogWrite(this->driverPins[5], this->speed);
}

void L298N::stop() {
    for(int i = 0; i < 4; i++){
        digitalWrite(this->driverPins[i], LOW);
    }
}

int L298N::getCurrentSpeed(){
    return this->speed;
}

void L298N::setSpeed(int newSpeed){
    this->speed = newSpeed;
}