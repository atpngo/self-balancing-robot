#include "robot.h"
#include "motor_pin_manager.h"

Robot::Robot(MotorPinManager pinsA, MotorPinManager pinsB) : 
    motorA(pinsA), 
    motorB(pinsB) 
{

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
    return imu.isReady();
}

void Robot::calculateAngles() {
    imu.calculateAngles();
}

