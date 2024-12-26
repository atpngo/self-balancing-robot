#include "imu.h"
#include "util.h"

IMU::IMU() {
    dmpReady = false;
    dataReady = false;
    oldYPR[0] = 0.0;
    oldYPR[1] = 0.0;
    oldYPR[2] = 0.0;
}

IMU::~IMU() {

}

void IMU::initialize() {
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // Initialize offsets (might have to tweak these)
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        calibrate();
    }
}

bool IMU::isReady() {
    return dmpReady;
}

void IMU::calibrate() {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
}


void IMU::calculateAngles() {
    dataReady = mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
    if (dataReady && isReady()) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

float IMU::getAngle(OrientationDimension dimension) {
    if (dataReady && isReady()) {
        oldYPR[dimension] = degToRad(ypr[dimension]);
    }
    return oldYPR[dimension];
}

float IMU::getYaw() {
    return getAngle(YAW);
}

float IMU::getPitch() {
    return getAngle(PITCH);
}

float IMU::getRoll() {
    return getAngle(ROLL);
}