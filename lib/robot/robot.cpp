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
