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

#include "MotorPID.h"

// Constructor: Initializes the MotorPID object with encoder and motor pins.
MotorPID::MotorPID(int encoderPinA, int encoderPinB, int motorPinPWM, int motorPinDirection)
    : encoder(encoderPinA, encoderPinB), motorPinPWM(motorPinPWM), motorPinDirection(motorPinDirection) {
    // Initialize other PID parameters here if needed
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;

    // Set up initial variables
    setpoint = 0;
    prevPosition = 0;
    integral = 0;

    // Configure motor pins
    pinMode(motorPinPWM, OUTPUT);
    pinMode(motorPinDirection, OUTPUT);

    // Initialize the PID controller
    outputLimits(-255, 255);
}

// Sets the desired position for the motor.
void MotorPID::setSetpoint(int target) {
    setpoint = target;
}

// Computes the PID control and updates the motor.
void MotorPID::compute() {
    // Read the current position from the encoder.
    int currentPosition = encoder.read();

    // Calculate the error between the desired and current positions.
    int error = setpoint - currentPosition;

    // Update the integral term.
    integral += error;

    // Calculate the derivative term.
    int derivative = currentPosition - prevPosition;

    // PID calculation
    int output = Kp * error + Ki * integral + Kd * derivative;

    // Apply output limits
    output = constrain(output, outputMin, outputMax);

    // Update motor based on the calculated output.
    if (output >= 0) {
        analogWrite(motorPinPWM, output);
        digitalWrite(motorPinDirection, HIGH);
    } else {
        analogWrite(motorPinPWM, -output);
        digitalWrite(motorPinDirection, LOW);
    }

    // Save current position for the next iteration
    prevPosition = currentPosition;
}

// Sets the PID constants.
void MotorPID::setPID(float p, float i, float d) {
    Kp = p;
    Ki = i;
    Kd = d;
}

// Sets the output limits for the motor PWM.
void MotorPID::outputLimits(int min, int max) {
    outputMin = min;
    outputMax = max;
}
