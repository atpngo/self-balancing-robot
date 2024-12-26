#ifndef _ROBOT
#define _ROBOT

#include "motor_pin_manager.h"
#include "motor.h"
#include "imu.h"

class Robot {
    public:
        Robot(MotorPinManager pinsA, MotorPinManager pinsB);
        ~Robot();
        Motor* getMotorA();
        Motor* getMotorB();
        IMU* getIMU();
        void initialize();
        bool isReady();
        void calculateAngles();

    private:
        Motor motorA;
        Motor motorB;
        IMU imu;
};

#endif // _ROBOT