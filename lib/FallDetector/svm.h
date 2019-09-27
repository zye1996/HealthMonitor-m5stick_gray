//
// Created by Zhenyi Ye on 2019-08-28.
//

#ifndef HEALTHMONITOR_SVM_H
#define HEALTHMONITOR_SVM_H

#include <Arduino.h>

#define VEC_DIM 8

#define SVM_TYPE c_svc
#define KERNEL_TYPE rbf
#define GAMMA 0.29999999999999999
#define NR_CLASS 2
#define TOTAL_SV 43

void scale(const float* sensor, float* scaledSensor);
uint8_t svm_predict(float sensor[]);
float svm_evaluate(int n_sv, float* coeffs, float* sv_class, float* sensors);
float linear_kernel(float* u, float* v);
float rbf_kernel(float* u, float* v);


#endif //HEALTHMONITOR_SVM_H
