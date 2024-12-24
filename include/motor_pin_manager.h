#ifndef _MOTOR_PIN_MANAGER
#define _MOTOR_PIN_MANAGER

struct MotorPinManager {
    // for hardware interrupts and reading encoder values
    int ENCODER_INTERRUPT;
    int ENCODER_SIGNAL;
    // send PMW signals to control motor speed and direction
    int MOTOR_IN_A;
    int MOTOR_IN_B;
    // enable pin to control speed
    int ENABLE;
};

#endif // _MOTOR_PIN_MANAGER