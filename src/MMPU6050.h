
#include <Wire.h>
#include "JJ_MPU6050_DMP_6Axis.h"
#include "MPin.h"
#define OUTPUT_READABLE_YAWPITCHROLL

//#define M_PI 3.14159265359

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

float euler[3]; // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void InitMPU()
{
    Wire.begin();
    Wire.setClock(400000);
    //mpu.initialize();
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20); //10,20,42,98,188
    mpu.setRate(4);                      // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    mpu.setSleepEnabled(false);

    delay(2000);
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(MPU_INT, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.println("Not read MPU6050");
    }

    delay(2000);
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

float dmpGetTheta()
{
    mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.resetFIFO(); // We always reset FIFO

    return( asin(-2*(q.x * q.z - q.w * q.y)) * RAD2GRAD);
    //return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);1
    //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll

    //yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    //pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
    //roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
}

float dmpGetPsi()
{
    //mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.resetFIFO(); // We always reset FIFO
    return (atan2(2*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z) * RAD2GRAD);
}