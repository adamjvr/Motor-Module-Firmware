/*
MIT License

Copyright (c) 2023 Adam Vadala-Roth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>  // Include Arduino standard library
#include <Encoder.h>  // Include Encoder library

class MotorPID {
public:
    // Constructor
    MotorPID(int encoderPinA, int encoderPinB, int pwmPin, int directionPin1);

    // Set the desired position for the motor
    void setSetpoint(int setpoint);

    // Compute PID and update motor state
    void compute();

    // Get the current direction of the motor
    int getDirection() const;

    // Get the current speed of the motor
    int getSpeed() const;

    // Check if the motor is stopped
    bool isStopped() const;

private:
    // Motor state struct
    struct MotorState {
        int speed;       // PWM value representing speed
        int direction;   // Two-bit value representing direction
        bool isStopped;  // Boolean indicating whether the motor is stopped
    };

    MotorState motorState;  // Current state of the motor

    // Encoder object for reading motor position
    Encoder encoder;

    // PID parameters
    double kp;
    double ki;
    double kd;

    // PID variables
    double previousError;
    double integral;

    // Motor control pins
    int pwmPin;
    int directionPin1;

    // Target position for PID control
    int setpoint;

    // Helper function to update motor direction based on PID output
    void updateDirection(int pidOutput);

    // Helper function to apply PWM output to control motor speed
    void applyPWM(int pwmValue);
};

#endif // MOTOR_PID_H

