#include "motor.h"
#include "motor_pin_manager.h"
#include "Arduino.h"
#include "util.h"

Motor::Motor(MotorPinManager pins) : controller(0.0, 0.0, 0.0) {
    this->pins = pins;
    pinMode(pins.ENCODER_INTERRUPT, INPUT);
    pinMode(pins.ENCODER_SIGNAL, INPUT);
    pinMode(pins.MOTOR_IN_A, OUTPUT);
    pinMode(pins.MOTOR_IN_B, OUTPUT);
    encoderValue = 0;
    this->type = MAIN;

    signal = 0.0;

    setPID(0.0, 0.0, 0.0);
    MIN_PMW = 28;    
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
    int speed = clamp(power + MIN_PMW, 0, 255); // TODO: modify min constant

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
    setMotorSpeed(speed);
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

void Motor::resetEncoder() {
    encoderValue = 0;
}

void Motor::spinToPosition(int targetPosition) {
    double signal = controller.getCommand(getEncoderValue(), targetPosition);
    Serial.print("Encoder: ");
    Serial.print(getEncoderValue());
    Serial.print(", Target: ");
    Serial.print(targetPosition);
    Serial.print(", Signal: ");
    Serial.println(signal);
    if (abs(signal) > 1) { // threshold
        spin(signal);
    }
}

void Motor::setPID(double Kp, double Ki, double Kd) {
    setKp(Kp);
    setKi(Ki);
    setKd(Kd);
}

double Motor::getKp() {
    return controller.getKp();
}

double Motor::getKi() {
    return controller.getKi();
}

double Motor::getKd() {
    return controller.getKd();
}

void Motor::setKp(double Kp) {
    controller.setKp(Kp);
}

void Motor::setKi(double Ki) {
    controller.setKi(Ki);
}

void Motor::setKd(double Kd) {
    controller.setKd(Kd);
}
