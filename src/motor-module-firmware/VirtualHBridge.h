#ifndef VIRTUAL_H_BRIDGE_H
#define VIRTUAL_H_BRIDGE_H

#include <Arduino.h>

class VirtualHBridge {
public:
    // Constructor to initialize the virtual H-bridge with control pins
    VirtualHBridge(int pin1, int pin2, int pinPWM);

    // Set the direction and speed of the motor
    void setMotor(int direction, int speed);

    // Stop the motor
    void stopMotor();

private:
    int pin1;    // Digital pin for direction control 1
    int pin2;    // Digital pin for direction control 2
    int pinPWM;  // PWM pin for speed control
};

#endif // VIRTUAL_H_BRIDGE_H
