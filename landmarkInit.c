#include <cblas.h>
#include <stdlib.h>
#include "headers/constants.h"
#include "headers/utils.h"

float* newLandmarkMean(float *pose, float *z, float *rotMat){
    float *mean = malloc(2 * sizeof(float));

    mean[0] = pose[1];
    mean[1] = pose[2];
    cblas_sgemv(CblasRowMajor, CblasTrans, 2, 2, 1.0, rotMat, 2, z, 1, 1.0, mean, 1);

    return mean;
}

float* newLandmarkCov(float *pose, float *rotMat){
    float *Q, *aux, *cov;
    
    aux = malloc(4 * sizeof(float));
    cov = malloc(4 * sizeof(float));

    Q = identityMatrix(2);

    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 2, 2, 2, 1.0, Q, 2, rotMat, 2, 0.0, aux, 2);
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, 1.0, rotMat, 2, aux, 2, 0.0, cov, 2);

    free(aux);
    free(Q);

    return cov;
}