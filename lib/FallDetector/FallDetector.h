//
// Created by Zhenyi Ye on 2019-08-26.
//

#ifndef HEALTHMONITOR_FALLDETECTOR_H
#define HEALTHMONITOR_FALLDETECTOR_H

#include "svm.h"
#include "type.h"


enum FallDetectorState {DETECTOR_IDLE, DETECTOR_PEAK_FOUND, DETECTOR_PEAK_SUSPECT};

struct AccFeature{
    float ammv;
    float idi;
    float mpi;
    float mvi;
    float pdi;
    float ari;
    float ffi;
    float sci;
};

class FallDetector {


    public:
        /* Constructor */
        FallDetector();

        void update();
        bool get_state();
        int get_result();
        void get_extracted_features(float buf[]);

        void set_callback(CallbackFunc alarm_callback, CallbackFunc non_alarm_callback);

    private:

        /* Feature Buffer */
        AccFeature acc_features;

        /* Indicate Peak Found */
        bool peak_found;

        /* Time Counter */
        long counter = 0;

        /* Result for Prediction */
        int prediction = -1;

        /* Index */
        uint16_t pt_i;
        uint16_t is_i;
        uint16_t ie_i;

        /* Threshold */
        const float PEAK_SETTLE_TIME = 2.5;
        const float PEAK_THRESHOLD   = 3.0;
        const float IMPACT_LOW_THRESHOLD = 0.8;
        const float IMPACT_HIGH_THRESHOLD = 1.5;
        const float IMPACT_END_TIME   = 1.0;
        const float IMPACT_START_TIME = -1.2;

        const float IVI_START_TIME = -0.5;

        const float PDI_THRESHOLD = 1.8;

        const float ARI_TIME_RANGE = 0.35;

        const float FFI_THRESHOLD = 0.8;
        const float FFI_TIME_RANGE = 0.2;

        const float SCI_LOW_THRESHOLD = 1;
        const float SCI_HIGH_THRESHOLD = 1.6;
        const float SCI_VALLEY_TIME   = 0.08;
        const float SCI_PEAK_TIME     = 0.2;
        const float SCI_TIME_INTERVAL = 0.2;
        const float SCI_TIME_RANGE = -2.2;

        FallDetectorState fall_detector_state;

        void _feature_extraction();
        float _get_ammv();
        float _get_idi();
        float _get_mpi();
        float _get_mvi();
        float _get_pdi();
        float _get_ari();
        float _get_ffi();
        float _get_sci();

        CallbackFunc _alarm;
        CallbackFunc _non_alarm;



};


#endif //HEALTHMONITOR_FALLDETECTOR_H
