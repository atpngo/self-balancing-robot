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
    this->type = MAIN;
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
    if (type == SECONDARY) {
        return -1*encoderValue;
    }
    return encoderValue;
}

void Motor::spin(LinearDirection direction, int power) {
    int speed = clamp(power, 0, 255);

    if (this->type == MAIN) {
        if (direction == FORWARD) { // MAIN + FOWARD -> CLOCKWISE
            setSpinDirection(CLOCKWISE);
        } else if (direction == BACKWARD) {
            setSpinDirection(COUNTERCLOCKWISE);
        }
    }
    else if (this->type == SECONDARY) {
        if (direction == FORWARD) { // SECONDARY + FOWARD -> COUNTER-CLOCKWISE
            setSpinDirection(COUNTERCLOCKWISE);
        } else if (direction == BACKWARD) {
            setSpinDirection(CLOCKWISE);
        }
    }

    // Set speed
    analogWrite(pins.ENABLE, speed);
}

void Motor::spin(int power) {
    if (power < 0) {
        spin(BACKWARD, abs(power));
    } else {
        spin(FORWARD, power);
    }
}

void Motor::stop() {
    digitalWrite(pins.MOTOR_IN_A, LOW);
    digitalWrite(pins.MOTOR_IN_B, LOW);
    analogWrite(pins.ENABLE, 0);
}

void Motor::setMode(MotorType type) {
    this->type = type;
}

void Motor::setSpinDirection(SpinDirection sd) {
    if (sd == COUNTERCLOCKWISE) {
        digitalWrite(pins.MOTOR_IN_A, LOW);
        digitalWrite(pins.MOTOR_IN_B, HIGH);
    }
    else if (sd == CLOCKWISE) {
        digitalWrite(pins.MOTOR_IN_A, HIGH);
        digitalWrite(pins.MOTOR_IN_B, LOW);
    }
}

void Motor::setMotorSpeed(int speed) {
    analogWrite(pins.ENABLE, speed);
}