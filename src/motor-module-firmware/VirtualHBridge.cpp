class VirtualHBridge {
public:
    // Constructor to initialize the virtual H-bridge with control pins
    VirtualHBridge(int pin1, int pin2, int pinPWM) : pin1(pin1), pin2(pin2), pinPWM(pinPWM) {
        // Initialize motor control pins
        pinMode(pin1, OUTPUT);
        pinMode(pin2, OUTPUT);
        pinMode(pinPWM, OUTPUT);

        // Initialize motor to stop state
        stopMotor();
    }

    // Set the direction and speed of the motor
    void setMotor(int direction, int speed) {
        // Ensure direction and speed are within valid range
        direction = constrain(direction, -1, 1);
        speed = constrain(speed, 0, 255);

        // Set direction
        digitalWrite(pin1, direction == 1);
        digitalWrite(pin2, direction == -1);

        // Set PWM speed
        analogWrite(pinPWM, speed);
    }

    // Stop the motor
    void stopMotor() {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
        analogWrite(pinPWM, 0);
    }

private:
    int pin1;    // Digital pin for direction control 1
    int pin2;    // Digital pin for direction control 2
    int pinPWM;  // PWM pin for speed control
};
