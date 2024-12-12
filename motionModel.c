#include <math.h>
#include <stdlib.h>
#include "headers/constants.h"

float derivBeta(float *u){
    float steer, derivBeta;

    steer = u[1];
    
    derivBeta = WHEELBASE / (pow(WHEELBASE * cos(steer), 2) + pow(LR * sin(steer), 2));

    return derivBeta;
}

float beta(float *u){
    float beta;

    beta = atan((LR / WHEELBASE) * tan(u[1]));

    return beta;
}

float* newPose(float *prevPose, float *u){
    float speed, betaU;
    float *newPose = malloc(3 * sizeof(float));

    speed = u[0];
    betaU = beta(u);

    newPose[0] = (speed * sin(betaU) / LR) * DELTA_T + prevPose[0];
    newPose[1] = (speed * cos(betaU + prevPose[0])) * DELTA_T + prevPose[1];
    newPose[2] = (speed * sin(betaU + prevPose[0])) * DELTA_T + prevPose[2];

    return newPose;
}