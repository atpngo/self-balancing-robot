#ifndef _MOTOR
#define _MOTOR

#include "motor_pin_manager.h"

class Motor {
    
    public:
        enum MotorDirection {
            CLOCKWISE, COUNTERCLOCKWISE
        };
        Motor(MotorPinManager pins);
        ~Motor();
        void updateEncoder();
        int getEncoderValue();
        void spin(MotorDirection direction, int power);
        void spin(int power);
        void stop();
    private:
        MotorPinManager pins;
        int encoderValue;
};

#endif // _MOTOR