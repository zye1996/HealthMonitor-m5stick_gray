//
// Created by Zhenyi Ye on 2019-08-16.
//

#include <M5Stack.h>
#include <math.h>
#include "WiFiSetup.h"
#include "ImuCalibration.h"
#include "FallDetector.h"
#include "type.h"

#define LedPin 19
#define IrPin 17
#define BuzzerPin 26
#define BtnPin 35

// Wifi client
extern char python_server[40];
extern char python_port[6];
WiFiClient client;  //NOLINT
uint16_t server_port;

// Sensor Buffer
SensorBuffer sensor_buffer; //NOLINT
// Fall Detector
FallDetector fallDetector;  //NOLINT

void M5_setup();
void alarm();
void non_alarm();


void setup(){

    // M5 Init
    M5_setup();

    // Fall Detector Setup
    fallDetector.set_callback(alarm, non_alarm);
}

void loop(){


    fallDetector.update();

    delay(48);

    // Check Button and Speaker State
    M5.update();

    if (M5.BtnA.wasPressed())
        digitalWrite(BuzzerPin,LOW);

}

void M5_setup(){

    M5.begin(true, true, true, true);
    pinMode(BuzzerPin, OUTPUT);
    // Imu Init
    M5.IMU.initMPU9250();
    M5.IMU.calibrateMPU9250(M5.IMU.gyroBias, M5.IMU.accelBias);

}

void alarm(){
    Serial.println("Fall..");
    for(int i=0;i<1000;i++){
        digitalWrite(BuzzerPin,HIGH);
        delay(1);
        digitalWrite(BuzzerPin,LOW);
        delay(1);
    }
}

void non_alarm(){
    Serial.println("Not fall..");
}
