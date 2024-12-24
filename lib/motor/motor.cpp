#include "motor.h"
#include "motor_pin_manager.h"
#include "Arduino.h"
#include "util.h"

Motor::Motor(MotorPinManager pins) {
    this->pins = pins;
    pinMode(pins.ENCODER_INTERRUPT, INPUT);
    pinMode(pins.ENCODER_SIGNAL, INPUT);
    pinMode(pins.MOTOR_IN_A, OUTPUT);
    pinMode(pins.MOTOR_IN_B, OUTPUT);
    encoderValue = 0;
}

Motor::~Motor() {

}

void Motor::updateEncoder() {
    if (digitalRead(pins.ENCODER_SIGNAL) == HIGH) {
        encoderValue++;
    } else {
        encoderValue--;
    }
}

int Motor::getEncoderValue() {
    return encoderValue;
}

void Motor::spin(MotorDirection direction, int power) {
    int speed = clamp(power, 0, 255);

    // Set direction
    switch (direction) {
        case CLOCKWISE:
            digitalWrite(pins.MOTOR_IN_A, HIGH);
            digitalWrite(pins.MOTOR_IN_B, LOW);
            break;
        case COUNTERCLOCKWISE:
            digitalWrite(pins.MOTOR_IN_A, LOW);
            digitalWrite(pins.MOTOR_IN_B, HIGH);
            break;
        default:
            break;
    }

    // Set speed
    analogWrite(pins.ENABLE, speed);
}

void Motor::spin(int power) {
    if (power < 0) {
        spin(CLOCKWISE, abs(power));
    } else {
        spin(COUNTERCLOCKWISE, power);
    }
}

void Motor::stop() {
    digitalWrite(pins.MOTOR_IN_A, LOW);
    digitalWrite(pins.MOTOR_IN_B, LOW);
    analogWrite(pins.ENABLE, 0);
}