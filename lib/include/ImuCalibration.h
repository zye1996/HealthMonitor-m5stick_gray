//
// Created by Zhenyi Ye on 2019-08-19.
//

#ifndef HEALTHMONITOR_IMUCALIBRATION_H
#define HEALTHMONITOR_IMUCALIBRATION_H

#define offsetAccX 1419
#define offsetAccY 510.5
#define offsetAccZ 386

#define gainAccX 4105.0
#define gainAccY 4112.5
#define gainAccZ 4120

#define offsetGyroX (-57)
#define offsetGyroY (-60)
#define offsetGyroZ 88

#define scaleGyroX 0.0558
#define scaleGyroY 0.0587
#define scaleGyroZ 0.0554

void calibrateAcc(float* raw_x, float* raw_y, float* raw_z);
void calibrateGyro(float* raw_x, float* raw_y, float* raw_z);
float readAccCal(float* accX, float* accY, float* accZ);
float readGyroCal(float* gyroX, float* gyroY, float* gyroZ);

#endif //HEALTHMONITOR_IMUCALIBRATION_H
