#ifndef _MOTOR
#define _MOTOR

#include "motor_pin_manager.h"

class Motor {
    
    public:
        enum MotorDirection {
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
    private:
        MotorPinManager pins;
        int encoderValue;
        MotorType type;
};

#endif // _MOTOR