#include "robot.h"
#include "motor_pin_manager.h"

Robot::Robot(MotorPinManager pinsA, MotorPinManager pinsB) : 
    motorA(pinsA), 
    motorB(pinsB) 
{
    isServoActivatedState = false;
    isArmedState = false;
    targetPosition = 0;
    targetPitch = 0.0;
}

Robot::~Robot() {

}

Motor* Robot::getMotorA() {
    return &motorA;
}

Motor* Robot::getMotorB() {
    return &motorB;
}

IMU* Robot::getIMU() {
    return &imu;
}

void Robot::initialize() {
    motorB.setMode(Motor::SECONDARY);
    imu.initialize();
}

bool Robot::isReady() {
    return isArmedState && imu.isReady();
}

bool Robot::isArmed() {
    return isArmedState;
}

bool Robot::isServoActivated() {
    return isServoActivatedState;
}

void Robot::calculateAngles() {
    imu.calculateAngles();
}

void Robot::setIsArmed(bool state) {
    isArmedState = state;
}

void Robot::arm() {
    setIsArmed(true);
    motorA.resetEncoder();
    motorB.resetEncoder();
}

void Robot::disarm() {
    setIsArmed(false);
    motorA.stop();
    motorB.stop();
    motorA.resetEncoder();
    motorB.resetEncoder();
}

void Robot::spinA(int power) {
    if (!isArmedState) {
        return;
    }
    motorA.spin(power);
}

void Robot::spinB(int power) {
    if (!isArmedState) {
        return;
    }
    motorB.spin(power);
}


void Robot::spinToPosition(int position) {
    motorA.spinToPosition(motorA.getEncoderValue() + position);
    motorB.spinToPosition(motorB.getEncoderValue() + position);
}

int Robot::getTargetPosition() {
    return targetPosition;
}

void Robot::setTargetPosition(int position) {
    targetPosition = position;
}

void Robot::moveToTargetPosition() {
    spinToPosition(targetPosition);
}


double Robot::getKp() {
    return motorA.getKp();
}

double Robot::getKi() {
    return motorA.getKi();
}

double Robot::getKd() {
    return motorA.getKd();
}


void Robot::setKp(double Kp) {
    motorA.setKp(Kp);
    motorB.setKp(Kp);
}

void Robot::setKi(double Ki) {
    motorA.setKi(Ki);
    motorB.setKi(Ki);
}

void Robot::setKd(double Kd) {
    motorA.setKd(Kd);
    motorB.setKd(Kd);
}

void Robot::setTargetPitch(float target) {
    targetPitch = target;
}

float Robot::getTargetPitch() {
    return targetPitch;
}
