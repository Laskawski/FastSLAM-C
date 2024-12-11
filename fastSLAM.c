#include <cblas.h>
#include <stdlib.h>

#include "headers/constants.h"
#include "headers/kalmanFilter.h"
#include "headers/landmarkInit.h"
#include "headers/motionModel.h"
#include "headers/particleWeight.h"
#include "headers/sampling.h"
#include "headers/structs.h"
#include "headers/utils.h"

void fastSLAM(Particle *particles, int numParticles, float *z, float *u){
    float *rotMat, *currPose, *measPred, *measCov, *measProb, *randU, *newMean, *newCov, weights[numParticles];
    int numLandmarks, corrLandmark, *sampledIndexes;
    Particle *particlePointer, *particleAuxPointer, *particlesAux;
    Landmark *landmarks, *landmarksAux, *landmarkPointer, *landmarkAuxPointer;

    particlesAux = particlesInit(numParticles);

    for(int i = 0; i < numParticles; i++){
        particlePointer = &particles[i];
        particleAuxPointer = &particlesAux[i];

        numLandmarks = particlePointer -> mapSize;
        landmarks = particlePointer -> landmarks;
        landmarksAux = particleAuxPointer -> landmarks;

        measProb = malloc((numLandmarks + 1) * sizeof(float));

        randU = standardNormalDist();
        randU[0] = SPEED_UNCERTAINTY * randU[0] + u[0];
        randU[1] = STEER_UNCERTAINTY * randU[1] + u[1];

        currPose = newPose(particlePointer -> pose, randU);
        cblas_scopy(3, currPose, 1, particleAuxPointer -> pose, 1);

        free(randU);

        rotMat = rotationMatrix(currPose[0]);

        //Calculate measurement for probabilty for each landmark
        for(int j = 0; j < numLandmarks; j++){
            landmarkPointer = &landmarks[j];
            measPred = predMeasurement(landmarkPointer -> mean, currPose, rotMat);
            measCov = measurementCovariance(landmarkPointer -> mean, landmarkPointer -> covariance, currPose, rotMat);
            measProb[j] = measurementProbability(measPred, measCov, z);
            free(measPred);
            free(measCov);
        }
        measProb[numLandmarks] = 0.5; //Probability threshold for new landmark
        corrLandmark = argMax(measProb, numLandmarks + 1); //Index of landmark with maximum measurement probability

        if(corrLandmark == numLandmarks) numLandmarks++;

        particleAuxPointer -> mapSize = numLandmarks;
        particleAuxPointer -> weight = measProb[corrLandmark];
        weights[i] = measProb[corrLandmark];

        for(int n = 0; n < numLandmarks; n++){
            landmarkPointer = &landmarks[n];
            landmarkAuxPointer = &landmarksAux[n];

            if (n == particlePointer -> mapSize){
                newMean = newLandmarkMean(currPose, z, rotMat);
                newCov = newLandmarkCov(currPose, rotMat);
                cblas_scopy(2, newMean, 1, landmarkAuxPointer -> mean, 1);
                cblas_scopy(4, newCov, 1, landmarkAuxPointer -> covariance, 1);
                free(newMean);
                free(newCov);
            }else{
                cblas_scopy(2, landmarkPointer -> mean, 1, landmarkAuxPointer -> mean, 1);
                cblas_scopy(4, landmarkPointer -> covariance, 1, landmarkAuxPointer -> covariance, 1);

                if(n == corrLandmark) correct(landmarkAuxPointer -> mean, landmarkAuxPointer -> covariance, z, currPose, rotMat);
            }
        }
        free(rotMat);
        free(measProb);
        free(currPose);
    }

    sampledIndexes = lowVarianceSampler(particlesAux, weights, numParticles);

    for(int i = 0; i < numParticles; i++) particlesCopy(&particlesAux[sampledIndexes[i]], &particles[i]);

    free(particlesAux);
    free(sampledIndexes);
}