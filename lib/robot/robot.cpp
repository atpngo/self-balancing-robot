#include "robot.h"
#include "motor_pin_manager.h"

Robot::Robot(MotorPinManager pinsA, MotorPinManager pinsB) : 
    motorA(pinsA), 
    motorB(pinsB) 
{
    isServoActivatedState = false;
    isArmedState = false;
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

void Robot::abort() {
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
