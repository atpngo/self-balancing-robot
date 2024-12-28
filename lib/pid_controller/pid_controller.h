#ifndef _PID_CONTROLLER
#define _PID_CONTROLLER

class PID_Controller {
    public:
        PID_Controller(double Kp, double Ki, double Kd);
        ~PID_Controller();
        void setBounds(double min, double max);
        void setDt(double dt);
        double getCommand(double value, double target);
        void reset();
        double getKp();
        double getKi();
        double getKd();
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);

    private:
        double Kp;
        double Ki;
        double Kd;
        double dt; // time slice
        double min;
        double max;
        double _integral;
        double _prev_error;
};

#endif // _PID_CONTROLLER