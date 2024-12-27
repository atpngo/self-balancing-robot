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
        bool getIsArmed();
        bool getIsServoActivated();

    private:
        Motor motorA;
        Motor motorB;
        IMU imu;
        bool isServoActivated;
        bool isArmed;
};

#endif // _ROBOT