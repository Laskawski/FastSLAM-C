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

Particle* fastSLAM(Particle *particles, int numParticles, float *z, float *u){
    float *prevPose, *rotMat, *currPose, *measPred, *measCov, *measProb, *randU, weights[numParticles];
    int numLandmarks, corrLandmark, *sampledIndexes;
    Particle *particlePointer, *particleAuxPointer, *particlesAux, *particlesFinal;
    Landmark *landmarks, *landmarksAux, *landmarkPointer, *landmarkAuxPointer;

    particlesAux = malloc(numParticles * sizeof(Particle));

    for(int i = 0; i < numParticles; i++){
        particlePointer = &particles[i];
        particleAuxPointer = &particlesAux[i];

        prevPose = particlePointer -> pose;

        numLandmarks = particlePointer -> mapSize;
        landmarks = particlePointer -> landmarks;
        landmarksAux = particleAuxPointer -> landmarks;

        measProb = malloc((numLandmarks + 1) * sizeof(float));

        randU = standardNormalDist();
        randU[0] = SPEED_UNCERTAINTY * randU[0] + u[0];
        randU[1] = STEER_UNCERTAINTY * randU[1] + u[1];
        currPose = newPose(prevPose, randU);

        rotMat = rotationMatrix(prevPose[0]);

        //Calculate measurement for probabilty for each landmark
        for(int j = 0; j < numLandmarks; j++){
            landmarkPointer = &landmarks[j];
            measPred = predMeasurement(landmarkPointer -> mean, prevPose, rotMat);
            measCov = measurementCovariance(landmarkPointer -> mean, landmarkPointer -> covariance, prevPose, rotMat);
            measProb[j] = measurementProbability(measPred, measCov, z);
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
                landmarkAuxPointer -> mean = newLandmarkMean(currPose, z, rotMat);
                landmarkAuxPointer -> covariance = newLandmarkCov(currPose, rotMat);
            }else{
                landmarkAuxPointer -> mean = landmarkPointer -> mean;
                landmarkAuxPointer -> covariance = landmarkPointer -> covariance;

                if(n == corrLandmark) correct(landmarkAuxPointer -> mean, landmarkAuxPointer -> covariance, z, currPose);
            }
        }
        free(measProb);
    }

    sampledIndexes = lowVarianceSampler(particlesAux, weights, numParticles);

    particlesFinal = malloc(numParticles * sizeof(Particle));

    for(int i = 0; i < particleAuxPointer -> mapSize; i++) particlesFinal[i] = particlesAux[sampledIndexes[i]];

    free(particlesAux);

    return particlesFinal;
}