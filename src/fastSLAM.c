#include <cblas.h>
#include <stdlib.h>

#include "headers/constants.h"
#include "headers/kalmanFilter.h"
#include "headers/landmarkInit.h"
#include "headers/motionModel.h"
#include "headers/particleWeight.h"
#include "headers/structs.h"
#include "headers/utils.h"

void fastSLAM(Particle *particles, int numParticles, int zLen, float *z, float *u){
    float *rotMat, *currPose, *measPred, *measCov, *randU, *newMean, *newCov, meas[2], weights[numParticles];
    int numLandmarks, corrLandmark, *sampledIndexes;
    Particle *particlePointer, *particleAuxPointer, *particlesAux;
    Landmark *landmarks, *landmarksAux, *landmarkPointer, *landmarkAuxPointer;

    particlesAux = particlesInit(numParticles);

    for(int i = 0; i < numParticles; i++){
        particleAuxPointer = &particlesAux[i];
        particlesCopy(&particles[i], particleAuxPointer);

        weights[i] = 1.0;

        numLandmarks = particleAuxPointer -> mapSize;
        landmarksAux = particleAuxPointer -> landmarks;

        randU = standardNormalDist();
        randU[0] = SPEED_UNCERTAINTY * randU[0] + u[0];
        randU[1] = STEER_UNCERTAINTY * randU[1] + u[1];

        currPose = newPose(particleAuxPointer -> pose, randU);
        cblas_scopy(3, currPose, 1, particleAuxPointer -> pose, 1);

        free(randU);

        rotMat = rotationMatrix(currPose[0]);

        for(int j = 0; j < zLen; j++){
            float measProb[numLandmarks + 1];

            meas[0] = z[2 * j];
            meas[1] = z[2 * j + 1];

            //Calculate measurement for probabilty for each landmark
            for(int k = 0; k < numLandmarks; k++){
                landmarkPointer = &landmarksAux[k];
                measPred = predMeasurement(landmarkPointer -> mean, currPose, rotMat);
                measCov = measurementCovariance(landmarkPointer -> mean, landmarkPointer -> covariance, currPose, rotMat);
                measProb[k] = measurementProbability(measPred, measCov, meas);
                free(measPred);
                free(measCov);
            }
            measProb[numLandmarks] = 0.1; //Probability threshold for new landmark
            corrLandmark = argMax(measProb, numLandmarks + 1); //Index of landmark with maximum measurement probability

            weights[i] = measProb[corrLandmark] * weights[i];

            if (corrLandmark == numLandmarks) {
                newMean = newLandmarkMean(currPose, meas, rotMat);
                newCov = newLandmarkCov(currPose, rotMat);
                cblas_scopy(2, newMean, 1, landmarksAux[numLandmarks].mean, 1);
                cblas_scopy(4, newCov, 1, landmarksAux[numLandmarks].covariance, 1);
                free(newMean);
                free(newCov);
                numLandmarks++;
            }else{
                correct(landmarksAux[corrLandmark].mean, landmarksAux[corrLandmark].covariance, meas, currPose, rotMat);
            }
        }
        free(currPose);
        free(rotMat);
        particleAuxPointer -> mapSize = numLandmarks;
        particleAuxPointer -> weight = weights[i];
    }

    sampledIndexes = lowVarianceSampler(particlesAux, weights, numParticles);

    for(int i = 0; i < numParticles; i++) particlesCopy(&particlesAux[sampledIndexes[i]], &particles[i]);

    freeParticles(particlesAux, numParticles);
    free(sampledIndexes);
}