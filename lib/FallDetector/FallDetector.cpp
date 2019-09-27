//
// Created by Zhenyi Ye on 2019-08-26.
//

#include <CircularBuffer.h>
#include <Arduino.h>
#include <algorithm>
#include <M5Stack.h>
#include "ImuCalibration.h"
#include "type.h"
#include "FallDetector.h"

extern SensorBuffer sensor_buffer;
extern M5Stack M5;

/* constructor */
FallDetector::FallDetector(){
    fall_detector_state = DETECTOR_IDLE;
    peak_found = false;
}

void FallDetector::update(){

    if(!peak_found){

        // IMU Data Buffer
        float accXf, accYf, accZf, gyroXf, gyroYf, gyroZf, acc_sum, gyro_sum;

        // Fetch Acc & Gyro Data
        acc_sum = readAccCal(&accXf, &accYf, &accZf);
        gyro_sum = readGyroCal(&gyroXf, &gyroYf, &gyroZf);

        sensor_buffer.totalAcc.push(acc_sum);
        Serial.println(acc_sum, 6);
        sensor_buffer.totalGyro.push(gyro_sum);
    }


    switch (fall_detector_state){
        case DETECTOR_IDLE:
            peak_found = false;
            if (!sensor_buffer.totalAcc.isFull()) break;         //wait until the buffer is full
            if (sensor_buffer.totalAcc.last() > PEAK_THRESHOLD){
                fall_detector_state = DETECTOR_PEAK_SUSPECT;
                M5.Lcd.fillScreen(YELLOW);
                Serial.println("Computing...");
            }
            break;
        case DETECTOR_PEAK_SUSPECT:
            if (sensor_buffer.totalAcc.last() > PEAK_THRESHOLD){
                counter = 0;
            }
            else if(++counter == PEAK_SETTLE_TIME * IMU_SAMPLING_RATE){
                fall_detector_state = DETECTOR_PEAK_FOUND;
                pt_i = sensor_buffer.totalAcc.size() - (1+counter); // get the pt_i
                ie_i = pt_i + IMPACT_END_TIME * IMU_SAMPLING_RATE; is_i = pt_i + IMPACT_START_TIME * IMU_SAMPLING_RATE;
                while(sensor_buffer.totalAcc[ie_i] < IMPACT_HIGH_THRESHOLD)  ie_i--;
                while(sensor_buffer.totalAcc[is_i] > IMPACT_LOW_THRESHOLD && is_i < pt_i) is_i++;
                while(sensor_buffer.totalAcc[is_i] < IMPACT_HIGH_THRESHOLD && is_i < pt_i) is_i++;
                Serial.printf("is: %d, ie: %d, pt: %d\n", is_i, ie_i, pt_i);
                Serial.printf("peak: %.5f\n", sensor_buffer.totalAcc[pt_i]);
                Serial.printf("is: %.5f\n", sensor_buffer.totalAcc[is_i]);
                counter = 0;
                peak_found = true;
                // TODO: here we stop sampling temporarily for a feature extraction
            }
            break;
        case DETECTOR_PEAK_FOUND:
            _feature_extraction();
            //get_extracted_features();
            float features[] = {acc_features.ammv,
                                acc_features.idi,
                                acc_features.mpi,
                                acc_features.mvi,
                                acc_features.pdi,
                                acc_features.ari,
                                acc_features.ffi,
                                acc_features.sci};
            prediction = svm_predict(features);

            /* Launch Alarm */
            if(prediction == 1)
                _alarm();
            else
                _non_alarm();

            fall_detector_state = DETECTOR_IDLE;
            break;
    }
}

int FallDetector::get_result() {
    int temp = prediction;
    prediction = -1;
    return temp;
}

bool FallDetector::get_state() {
    return peak_found;
}

void FallDetector::get_extracted_features(float buf[]) {
    Serial.printf("features: %f %f %f %f %f %f %f %f\n",
            acc_features.ammv,
            acc_features.idi,
            acc_features.mpi,
            acc_features.mvi,
            acc_features.pdi,
            acc_features.ari,
            acc_features.ffi,
            acc_features.sci);
    *(buf+0) = acc_features.ammv; *(buf+1) = acc_features.idi; *(buf+2) = acc_features.mpi; *(buf+3) = acc_features.mvi;
    *(buf+4) = acc_features.pdi; *(buf+5) = acc_features.ari; *(buf+6) = acc_features.ffi; *(buf+7) = acc_features.sci;
}

void FallDetector::_feature_extraction(){
    acc_features.ammv = _get_ammv();
    acc_features.idi  = _get_idi();
    acc_features.ari  = _get_ari();
    acc_features.ffi  = _get_ffi();
    acc_features.mpi  = _get_mpi();
    acc_features.pdi  = _get_pdi();
    acc_features.sci  = _get_sci();
    acc_features.mvi  = _get_mvi();
}

void FallDetector::set_callback(CallbackFunc alarm_callback, CallbackFunc non_alarm_callback) {
    _alarm = alarm_callback;
    _non_alarm = non_alarm_callback;
}

/* Function get Average Absolute Magnitude Variation */
float FallDetector::_get_ammv(){
    float ret_val = 0;
    for (int i = is_i; i <= ie_i; i++){
        Serial.printf("%.5f\n", sensor_buffer.totalAcc[i]);
        ret_val += fabs(sensor_buffer.totalAcc[i] - sensor_buffer.totalAcc[i+1]);
    }
    ret_val = ret_val / (ie_i - is_i + 1);
    Serial.printf("ammv: %f", ret_val);
    return ret_val;
}

/* Function to get Impact Duration Index */
float FallDetector::_get_idi() {
    float ret_val = (ie_i - is_i) / (float)IMU_SAMPLING_RATE;
    Serial.printf("idi: %f", ret_val);
    return  ret_val;
}

/* Function to get Maximum Peak Index */
float FallDetector::_get_mpi() {
    float max_peak = sensor_buffer.totalAcc[pt_i];
    for (int i = is_i; i <= ie_i; i++){
        if (sensor_buffer.totalAcc[i] > max_peak)
            max_peak = sensor_buffer.totalAcc[i];
    }
    Serial.printf("mpi: %f", max_peak);
    return max_peak;
}

/* Function to get Minimum Valley Index */
float FallDetector::_get_mvi() {
    float low_valley = infinity();
    for (int i = is_i+IMU_SAMPLING_RATE*IVI_START_TIME; i<=ie_i; i++){
        if (sensor_buffer.totalAcc[i] < low_valley)
            low_valley = sensor_buffer.totalAcc[i];
    }
    Serial.printf("mvi: %f", low_valley);
    return low_valley;
}

/* Function to get Peak Duration Index */
float FallDetector::_get_pdi() {
    uint16_t s=pt_i, e=pt_i;
    while (sensor_buffer.totalAcc[s] > PDI_THRESHOLD) s--;
    while (sensor_buffer.totalAcc[e] > PDI_THRESHOLD) e++;
    float ret_val = (e-s) / (float)IMU_SAMPLING_RATE;
    Serial.printf("pdi: %f", ret_val);
    return ret_val;
}

/* Function to get Activity Ratio Index */
float FallDetector::_get_ari() {
    uint16_t count = 0;
    uint16_t s = (is_i+ie_i)/2-ARI_TIME_RANGE*IMU_SAMPLING_RATE;
    uint16_t e = (is_i+ie_i)/2+ARI_TIME_RANGE*IMU_SAMPLING_RATE;
    for(int i = s ; i <= e; i++){
        if (sensor_buffer.totalAcc[i] > 1.3 || sensor_buffer.totalAcc[i] < 0.8)
            count++;
    }
    float ret_val = count / (float)(e-s+1);
    Serial.printf("ari: %f", ret_val);
    return ret_val;
}

/* Function to get Free-Fall Index */
float FallDetector::_get_ffi() {
    uint16_t e = pt_i - 1;
    while(sensor_buffer.totalAcc[e] >= FFI_THRESHOLD && e > pt_i-FFI_TIME_RANGE*IMU_SAMPLING_RATE) e--;
    uint16_t s = e - FFI_TIME_RANGE*IMU_SAMPLING_RATE;
    float ret_val = 0.0;
    for(int i = s; i<=e; i++){
        ret_val += sensor_buffer.totalAcc[i];
    }
    ret_val = ret_val / (e-s+1);
    Serial.printf("ffi: %f", ret_val);
    return ret_val;
}

/* Function to get Step Count Index */
float FallDetector::_get_sci() {
    uint16_t step = 0;
    uint16_t l_step = pt_i+SCI_TIME_RANGE*IMU_SAMPLING_RATE;
    while (l_step < pt_i){
        boolean valley_found = false;
        if (sensor_buffer.totalAcc[l_step] < SCI_LOW_THRESHOLD){
            uint16_t l_step_prev = l_step;
            do{
                l_step++;
            }while(sensor_buffer.totalAcc[l_step] < SCI_LOW_THRESHOLD);
            if(l_step-l_step_prev-1 >= int(SCI_VALLEY_TIME * IMU_SAMPLING_RATE))
                valley_found = true;
        }
        else
            l_step++;

        if(valley_found){
            uint16_t l_step_prev = l_step-1;
            for (;l_step <= (int)(l_step_prev+SCI_PEAK_TIME*IMU_SAMPLING_RATE); l_step++){
                if (sensor_buffer.totalAcc[l_step] > SCI_HIGH_THRESHOLD){
                    step++;
                    l_step = (int)(l_step_prev+SCI_TIME_INTERVAL*IMU_SAMPLING_RATE)+1;
                    break;
                }
            }
        }
    }
    Serial.printf("sci: %f", (float)step);
    return (float)step;
}