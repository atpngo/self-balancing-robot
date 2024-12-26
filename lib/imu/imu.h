// A wrapper class around the I2Cdev MPU6050
#ifndef _IMU
#define _IMU

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

class IMU {
    public:
        IMU();
        ~IMU();
        void initialize(); // initialize I2C, digital motion processor, and calibrate 
        bool isReady();
        void calculateAngles();
        
        enum OrientationDimension {
            YAW, PITCH, ROLL
        };

        float getAngle(OrientationDimension dimension);
        float getYaw();
        float getPitch();
        float getRoll();


    private:
        void calibrate(); // assumes we are upright at 0 deg when this is called
        MPU6050 mpu;
        // MPU control/status vars
        bool dmpReady;  // set true if DMP init was successful
        bool dataReady;
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        // keep track of old values
        float oldYPR[3];
};

#endif // _IMU