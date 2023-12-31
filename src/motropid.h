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

#ifndef MOTORPID_H
#define MOTORPID_H

#include <Arduino.h>
#include <Encoder.h>

class MotorPID {
public:
    // Constructor: Initializes the MotorPID object with encoder and motor pins.
    MotorPID(int encoderPinA, int encoderPinB, int motorPinPWM, int motorPinDirection);

    // Sets the desired position for the motor.
    void setSetpoint(int target);

    // Computes the PID control and updates the motor.
    void compute();

    // Sets the PID constants.
    void setPID(float p, float i, float d);

    // Sets the output limits for the motor PWM.
    void outputLimits(int min, int max);

private:
    // Encoder object for reading motor position.
    Encoder encoder;

    // Motor control pins.
    int motorPinPWM;
    int motorPinDirection;

    // Desired position for the motor.
    int setpoint;

    // Previous position for computing the derivative.
    int prevPosition;

    // Integral term for PID control.
    int integral;

    // PID constants.
    float Kp, Ki, Kd;

    // Output limits for the motor PWM.
    int outputMin, outputMax;
};

#endif // MOTORPID_H
