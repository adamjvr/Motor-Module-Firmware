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

// Constructor
MotorPID::MotorPID(int encoderPinA, int encoderPinB, int pwmPin, int directionPin1)
    : encoder(encoderPinA, encoderPinB), pwmPin(pwmPin), directionPin1(directionPin1) {
    // Initialize motor state
    motorState.speed = 0;
    motorState.direction = 0;
    motorState.isStopped = true;

    // Initialize PID parameters
    kp = 0.1;
    ki = 0.01;
    kd = 0.1;

    // Initialize PID variables
    previousError = 0;
    integral = 0;

    // Initialize motor control pins
    pinMode(pwmPin, OUTPUT);
    pinMode(directionPin1, OUTPUT);

    // Set initial direction to brake
    digitalWrite(directionPin1, LOW);

    // Set initial PWM value to stop the motor
    analogWrite(pwmPin, 0);
}

// Set the desired position for the motor
void MotorPID::setSetpoint(int setpoint) {
    this->setpoint = setpoint;
}

// Compute PID and update motor state
void MotorPID::compute() {
    // Calculate error and update integral
    double error = setpoint - encoder.read();
    integral += error;

    // Calculate PID output
    double pidOutput = kp * error + ki * integral + kd * (error - previousError);

    // Update motor direction based on PID output
    updateDirection(pidOutput);

    // Apply PWM output to control motor speed
    applyPWM(abs(static_cast<int>(pidOutput)));

    // Update previous error
    previousError = error;

    // Update motor state
    motorState.isStopped = (pidOutput == 0);
    motorState.speed = static_cast<int>(abs(pidOutput));
}

// Get the current direction of the motor
int MotorPID::getDirection() const {
    return motorState.direction;
}

// Get the current speed of the motor
int MotorPID::getSpeed() const {
    return motorState.speed;
}

// Check if the motor is stopped
bool MotorPID::isStopped() const {
    return motorState.isStopped;
}

// Helper function to update motor direction based on PID output
void MotorPID::updateDirection(int pidOutput) {
    if (pidOutput > 0) {
        // Moving forward
        motorState.direction = 1;
        digitalWrite(directionPin1, HIGH);
    } else if (pidOutput < 0) {
        // Moving backward
        motorState.direction = 2;
        digitalWrite(directionPin1, LOW);
    } else {
        // Brake or coast
        motorState.direction = 0;
        digitalWrite(directionPin1, LOW);  // Set to brake for simplicity
    }
}

// Helper function to apply PWM output to control motor speed
void MotorPID::applyPWM(int pwmValue) {
    analogWrite(pwmPin, pwmValue);
}
