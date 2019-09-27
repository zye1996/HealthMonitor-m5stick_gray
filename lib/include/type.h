//
// Created by Zhenyi Ye on 2019-08-26.
//

#ifndef HEALTHMONITOR_TYPE_H
#define HEALTHMONITOR_TYPE_H

#include <CircularBuffer.h>
#include "constant.h"

struct SensorBuffer
{
    CircularBuffer<float, IMU_BUFFER_TIME*IMU_SAMPLING_RATE> totalAcc;
    CircularBuffer<float, IMU_BUFFER_TIME*IMU_SAMPLING_RATE> totalGyro;
};

typedef void (*CallbackFunc)();


#endif //HEALTHMONITOR_TYPE_H
