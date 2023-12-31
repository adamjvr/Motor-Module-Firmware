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
#include "VirtualHBridge.h"


// Motor control pins
const int motorEncoderPinA = 2;    // Replace with your actual encoder pin A
const int motorEncoderPinB = 3;    // Replace with your actual encoder pin B
const int motorPWMPin = 9;         // Replace with your actual motor PWM pin
const int motorDirectionPin1 = 8;  // Replace with your actual motor direction pin 1
const int motorDirectionPin2 = 7;  // Replace with your actual motor direction pin 2
const int motorBrakePin = 6;       // Replace with your actual motor brake pin

// MotorPID object
MotorPID motor(motorEncoderPinA, motorEncoderPinB, motorPWMPin, motorDirectionPin1);

// VirtualHBridge object
VirtualHBridge hBridge(motorDirectionPin1, motorDirectionPin2, motorPWMPin);

// Uncomment the desired control input method
//#define USB_CONTROL
//#define SPI_CONTROL
//#define UART_CONTROL
#define DIGITAL_CONTROL

#ifdef USB_CONTROL
void setup() {
    // Initialize Serial communication for receiving commands over USB
    Serial.begin(9600);

    // Set initial motor setpoint
    motor.setSetpoint(0);
}

void loop() {
    // Check for incoming commands over USB
    if (Serial.available() > 0) {
        char command = Serial.read();

        // Perform action based on the received command
        switch (command) {
            case 'F':
                // Move the motor forward
                motor.setSetpoint(100);  // Set desired position
                break;
            case 'B':
                // Move the motor backward
                motor.setSetpoint(-100); // Set desired position
                break;
            case 'S':
                // Stop the motor
                motor.setSetpoint(0);    // Set desired position
                break;
            default:
                // Invalid command
                break;
        }
    }

    // Perform PID control to update the motor
    motor.compute();

    // Update the virtual H-bridge based on motor control signals
    hBridge.setMotor(motor.getDirection(), motor.getSpeed());

    // Add any additional logic as needed
}
#endif

#ifdef SPI_CONTROL
void setup() {
    // Initialize SPI communication
    SPI.begin();

    // Set initial motor setpoint
    motor.setSetpoint(0);
}

void loop() {
    // Check for incoming SPI data
    if (SPI.available()) {
        char command = SPI.transfer(0);

        // Perform action based on the received command
        switch (command) {
            case 'F':
                // Move the motor forward
                motor.setSetpoint(100);  // Set desired position
                break;
            case 'B':
                // Move the motor backward
                motor.setSetpoint(-100); // Set desired position
                break;
            case 'S':
                // Stop the motor
                motor.setSetpoint(0);    // Set desired position
                break;
            default:
                // Invalid command
                break;
        }
    }

    // Perform PID control to update the motor
    motor.compute();

    // Update the virtual H-bridge based on motor control signals
    hBridge.setMotor(motor.getDirection(), motor.getSpeed());

    // Add any additional logic as needed
}
#endif

#ifdef UART_CONTROL
void setup() {
    // Initialize Serial communication for receiving commands over UART
    Serial.begin(9600);

    // Set initial motor setpoint
    motor.setSetpoint(0);
}

void loop() {
    // Check for incoming commands over UART
    if (Serial.available() > 0) {
        char command = Serial.read();

        // Perform action based on the received command
        switch (command) {
            case 'F':
                // Move the motor forward
                motor.setSetpoint(100);  // Set desired position
                break;
            case 'B':
                // Move the motor backward
                motor.setSetpoint(-100); // Set desired position
                break;
            case 'S':
                // Stop the motor
                motor.setSetpoint(0);    // Set desired position
                break;
            default:
                // Invalid command
                break;
        }
    }

    // Perform PID control to update the motor
    motor.compute();

    // Update the virtual H-bridge based on motor control signals
    hBridge.setMotor(motor.getDirection(), motor.getSpeed());

    // Add any additional logic as needed
}
#endif

#ifdef DIGITAL_CONTROL
void setup() {
    // Set initial motor setpoint
    motor.setSetpoint(0);

    // Configure motor control pins for digital control
    pinMode(motorDirectionPin1, OUTPUT);
    pinMode(motorDirectionPin2, OUTPUT);
    pinMode(motorBrakePin, OUTPUT);
}

void loop() {
    // Example: Digital control with PWM for speed
    digitalWrite(motorDirectionPin1, HIGH);  // Set direction
    digitalWrite(motorDirectionPin2, LOW);   // Set direction
    analogWrite(motorPWMPin, 200);            // Set speed (PWM)
    digitalWrite(motorBrakePin, LOW);        // Release brake

    // Update the virtual H-bridge based on motor control signals
    hBridge.setMotor(motor.getDirection(), motor.getSpeed());

    // Add any additional logic as needed
}
#endif
