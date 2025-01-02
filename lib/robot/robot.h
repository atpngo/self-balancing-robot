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
        void disarm();

        // Controls
        void spinA(int power);
        void spinB(int power);

        void spinToPosition(int position);

        int getTargetPosition();
        void setTargetPosition(int position);
        void moveToTargetPosition();

        double getKp();
        double getKi();
        double getKd();

        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);

        void setTargetPitch(float target);
        float getTargetPitch();


    private:
        void setIsArmed(bool state);
        Motor motorA;
        Motor motorB;
        IMU imu;
        bool isServoActivatedState;
        bool isArmedState;
        int targetPosition;
        float targetPitch;
};

#endif // _ROBOT