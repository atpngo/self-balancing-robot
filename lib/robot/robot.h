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
        bool isArmed();
        bool isServoActivated();
        void arm();
        void abort();

        // Controls
        void spinA(int power);
        void spinB(int power);

    private:
        void setIsArmed(bool state);
        Motor motorA;
        Motor motorB;
        IMU imu;
        bool isServoActivatedState;
        bool isArmedState;
};

#endif // _ROBOT