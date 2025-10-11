#ifndef L298N_H
#define L298N_H

#include <Arduino.h>
#include "config.h"


// driverPins[0] = IN1 
// driverPins[1] = IN2 
// driverPins[2] = IN3 
// driverPins[3] = IN4 
// driverPins[4] = ENA 
// driverPins[5] = ENB 

/* Note: Invalid pins in esp32 = {
    0, 1, 3, 6, 7, 8, 9, 10, 11, 12, 15, 
}*/




class L298N
{
    private:
        const uint8_t driverPins[6] = {IN1, IN2, IN3, IN4, ENA, ENB};
        int stateMotorA;     // 0: static, 1: forward, -1: backward
        int stateMotorB;     // 0: static, 1: forward, -1: backward
        int speed = DEFAULT_SPEED;

    public:
        void moveForward();
        void moveBackward();
        void rotateRight();
        void rotateLeft();
        void stop();
        void initialMotors();
        void setSpeed(int newSpeed);
        int getCurrentSpeed();
};

#endif