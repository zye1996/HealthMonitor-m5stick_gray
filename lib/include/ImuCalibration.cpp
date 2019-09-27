//
// Created by Zhenyi Ye on 2019-08-19.
//

#include "ImuCalibration.h"
#include <M5Stack.h>
#include <math.h>

void calibrateAcc(float* raw_x, float* raw_y, float* raw_z){
    *raw_x = (*raw_x - offsetAccX) / gainAccX;
    *raw_y = (*raw_y - offsetAccY) / gainAccY;
    *raw_z = (*raw_z - offsetAccZ) / gainAccZ;
}


void calibrateGyro(float* raw_x, float* raw_y, float* raw_z){
    *raw_x = (*raw_x - offsetGyroX) * scaleGyroX;
    *raw_y = (*raw_y - offsetGyroY) * scaleGyroY;
    *raw_z = (*raw_z - offsetGyroZ) * scaleGyroZ;
}

float readAccCal(float* accX, float* accY, float* accZ){

    M5.IMU.readAccelData(M5.IMU.accelCount);
    M5.IMU.getAres(); // get accelerometer scales saved to "aRes"
    M5.IMU.ax = (float)M5.IMU.accelCount[0] * M5.IMU.aRes - M5.IMU.accelBias[0];
    M5.IMU.ay = (float)M5.IMU.accelCount[1] * M5.IMU.aRes - M5.IMU.accelBias[1];
    M5.IMU.az = (float)M5.IMU.accelCount[2] * M5.IMU.aRes -  M5.IMU.accelBias[2];

    Serial.printf("x: %.2f y: %.2f z: %.2f\n", M5.IMU.ax, M5.IMU.ay, M5.IMU.az);

    *accX = M5.IMU.ax; *accY = M5.IMU.ay; *accZ = M5.IMU.az;
    return sqrtf((*accX)*(*accX) + (*accY)*(*accY) + (*accZ)*(*accZ));
}


float readGyroCal(float* gyroX, float* gyroY, float* gyroZ){

    M5.IMU.readGyroData(M5.IMU.gyroCount);  // Read the x/y/z adc values
    M5.IMU.getGres(); // get Gyro scales saved to "gRes"
    M5.IMU.gx = (float)M5.IMU.gyroCount[0] * M5.IMU.gRes;
    M5.IMU.gy = (float)M5.IMU.gyroCount[1] * M5.IMU.gRes;
    M5.IMU.gz = (float)M5.IMU.gyroCount[2] * M5.IMU.gRes;

    *gyroX = M5.IMU.gx; *gyroY = M5.IMU.gy; *gyroZ = M5.IMU.gz;
    return sqrtf((*gyroX)*(*gyroX) + (*gyroY)*(*gyroY) + (*gyroZ)*(*gyroZ));
}

