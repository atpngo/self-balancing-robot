#include "pid_controller.h"
#include "util.h"

PID_Controller::PID_Controller(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    setBounds(-255, 255);
}

PID_Controller::~PID_Controller() {

}

void PID_Controller::setBounds(double min, double max) {
    this->min = min;
    this->max = max;
}

double PID_Controller::getCommand(double value, double target) {
    double error = value-target;
    
    // Proportion
    double Pout = Kp * error;
    // Integral
    _integral += error;
    double Iout = Ki * _integral;
    // Derivative
    double _derivative = error-_prev_error;
    double Dout = Kd * _derivative;
    _prev_error = error;

    double pid = Pout + Iout + Dout;
    return clamp(pid, min, max);
}


void PID_Controller::reset() {
    _integral = 0.0;
    _prev_error = 0.0;
}