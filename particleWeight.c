#include <cblas.h>
#include <math.h>
#include <stdlib.h>

float measurementProbability(float *measPred, float *measCov, float *z){
    float detMeasCov, exponent, *auxVect, *measDiff;

    auxVect = malloc(2 * sizeof(float));
    measDiff = malloc(2 * sizeof(float));

    detMeasCov = measCov[0] * measCov[3] - measCov[1] * measCov[2];
    detMeasCov = sqrt(2 * M_PI * detMeasCov);

    cblas_scopy(2, z, 1, measDiff, 1);
    cblas_saxpy(2, -1.0, measPred, 1, measDiff, 1);
    cblas_sgemv(CblasRowMajor, CblasNoTrans, 2, 2, 1.0, measCov, 2, measDiff, 1, 0.0, auxVect, 1);
    exponent = -cblas_sdot(2, measDiff, 1, auxVect, 1);

    free(auxVect);
    free(measDiff);

    return detMeasCov * exp(exponent);
}