#include <cblas.h>
#include <lapacke.h>
#include "headers/constants.h"
#include "headers/utils.h"

float* predMeasurement(float *mean, float *pose, float *rotMat){
    float *relativePos = malloc(2 * sizeof(float));
    float *predMeasurement = malloc(2 * sizeof(float));

    relativePos[0] = mean[0] - pose[1];
    relativePos[1] = mean[1] - pose[2];
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 2, 2, 1.0, rotMat, 2, relativePos, 1, 0.0, predMeasurement, 1); //predMeasurement <- rotMat@relativePos

    free(relativePos);

    return predMeasurement;
}

float* measurementCovariance(float *mean, float *covariance, float *pose, float *rotMat){
    float *auxMat, *measurementCovariance;

    auxMat = malloc(4 * sizeof(float));

    measurementCovariance = diagMatrix(2, 0.6);

    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, 1.0, rotMat, 2, covariance, 2, 0.0, auxMat, 2); //auxMat <- rotMat@covariance
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 2, 2, 2, 1.0, auxMat, 2, rotMat, 2, 1.0, measurementCovariance, 2); //measurementCovariance <- auxMat@rotMat^T

    free(auxMat);

    return measurementCovariance;
}

float* kalmanGain(float *mean, float *covariance, float *pose, float *rotMat, int *ipiv){
    float *invMeasurementCov;
    float *auxMat1 = malloc(4 * sizeof(float));
    float *auxMat2 = malloc(4 * sizeof(float));

    invMeasurementCov = measurementCovariance(mean, covariance, pose, rotMat);

    LAPACKE_sgetri(LAPACK_ROW_MAJOR, 2, invMeasurementCov, 2, ipiv); //invert measurementCovariance
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 2, 2, 2, 1.0, covariance, 2, rotMat, 2, 0.0, auxMat1, 2); //auxMat <- covariance@rotMat
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, 1.0, auxMat1, 2, invMeasurementCov, 2, 0.0, auxMat2, 2); //auxMat <- auxMat@invMeasurementCov

    free(invMeasurementCov);
    free(auxMat1);

    return auxMat2;
}

void correct(float* mean, float* covariance, float* z, float* pose, float *rotMat){
    float *auxMat, *K, *predMeas;
    float *measDiff = malloc(2 * sizeof(float));
    int *ipiv = malloc(2 * sizeof(int));

    ipiv[0] = 1;
    ipiv[1] = 2;
    auxMat = diagMatrix(2, 1.0);
    K = kalmanGain(mean, covariance, pose, rotMat, ipiv);
    predMeas = predMeasurement(mean, pose, rotMat);

    //Calculate difference between predicted and actual measurement
    measDiff[0] = z[0] - predMeas[0];
    measDiff[1] = z[1] - predMeas[1];

    //Correct mean
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 2, 2, 1.0, K, 2, measDiff, 1, 1.0, mean, 1); //mean <- K@measDiff
    
    //Correct covariance
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, -1.0, K, 2, rotMat, 2, 1.0, auxMat, 2); //auxMat <- auxMat - K@rotMat
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 2, 2, 2, 1.0, auxMat, 2, covariance, 2, 0.0, covariance, 2); //covariance <- auxMat@covariance
    
    free(auxMat);
    free(K);
    free(predMeas);
    free(measDiff);
    free(ipiv);
}