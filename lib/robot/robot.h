#ifndef _ROBOT
#define _ROBOT

#include "motor_pin_manager.h"
#include "motor.h"

class Robot {
    public:
        Robot(MotorPinManager pinsA, MotorPinManager pinsB);
        ~Robot();
        Motor* getMotorA();
        Motor* getMotorB();

    private:
        Motor motorA;
        Motor motorB;        
        
        

};

#endif // _ROBOT