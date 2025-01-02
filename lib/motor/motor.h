#ifndef _MOTOR
#define _MOTOR

#include "motor_pin_manager.h"
#include "pid_controller.h"

class Motor {
    
    public:
        enum SpinDirection {
            CLOCKWISE, COUNTERCLOCKWISE
        };
        enum MotorType {
            MAIN, SECONDARY
        };
        enum LinearDirection {
            FORWARD, BACKWARD
        };
        Motor(MotorPinManager pins);
        ~Motor();
        void updateEncoder();
        int getEncoderValue();
        void spin(LinearDirection direction, int power);
        void spin(int power);
        void stop();
        void setMode(MotorType type);
        void resetEncoder();
        void spinToPosition(int targetPosition);
        void setPID(double Kp, double Ki, double Kd);
        double getKp();
        double getKi();
        double getKd();
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
    private:
        void setSpinDirection(SpinDirection sd);
        void setMotorSpeed(int speed);
        MotorPinManager pins;
        int encoderValue;
        MotorType type;
        PID_Controller controller;
        double signal;
};

#endif // _MOTOR