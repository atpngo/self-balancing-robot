#include "robot.h"
#include "motor_pin_manager.h"

Robot::Robot(MotorPinManager pinsA, MotorPinManager pinsB) : 
    motorA(pinsA), 
    motorB(pinsB) 
{
    isServoActivated = false;
    isArmed = false;
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
    return isArmed && imu.isReady();
}

bool Robot::getIsArmed() {
    return isArmed;
}

bool Robot::getIsServoActivated() {
    return isServoActivated;
}

void Robot::calculateAngles() {
    imu.calculateAngles();
}

