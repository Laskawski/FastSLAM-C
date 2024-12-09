#include <cblas.h>
#include <stdlib.h>

typedef struct Landmark{
    float *mean;
    float *covariance;
    int counter;
}Landmark;

typedef struct Particle{
    float weight, *pose;
    int mapSize;
    struct Landmark *landmarks;
}Particle;

void landmarksCopy(Landmark landmarkO, Landmark landmarkD){
    Landmark *landmarkPointerO, *landmarkPointerD;

    landmarkPointerO = &landmarkO;
    landmarkPointerD = &landmarkD;

    cblas_scopy(2, landmarkPointerO -> mean, 1, landmarkPointerD -> mean, 1);
    cblas_scopy(4, landmarkPointerO -> covariance, 1, landmarkPointerD -> covariance, 1);
    landmarkPointerD -> counter = landmarkPointerO -> counter;
}

Particle* particlesInit(int numParticles){
    Landmark *landmark;
    Particle *particles, *particlePointer;
    float *pose, *mean, *covariance;

    particles = malloc(numParticles * sizeof(Particle));

    for(int n = 0; n < numParticles; n++){
        pose = malloc(3 * sizeof(float));
        pose[0] = 0.0;
        pose[1] = 0.0;
        pose[2] = 0.0;

        mean = malloc(2 * sizeof(float));
        mean[0] = 0.0;
        mean[1] = 0.0;

        covariance = malloc(2 * sizeof(float));
        covariance[0] = 0.0;
        covariance[1] = 0.0;
        covariance[2] = 0.0;
        covariance[3] = 0.0;

        landmark = malloc(sizeof(Landmark));
        landmark -> mean = mean;
        landmark -> covariance = covariance;
        landmark -> counter = 0;

        particlePointer = &particles[n];
        particlePointer -> weight = 0.0;
        particlePointer -> pose = pose;
        particlePointer -> mapSize = 0;
        particlePointer -> landmarks = landmark;
    }

    return particles;
}

void particlesCopy(Particle particleO, Particle particleD){
    Particle *particlePointerO, *particlePointerD;

    particlePointerO = &particleO;
    particlePointerD = &particleD;
    particlePointerD -> weight = particlePointerO -> weight;
    particlePointerD -> pose = particlePointerO -> pose;
    particlePointerD -> mapSize = particlePointerO -> mapSize;
    particlePointerD -> landmarks = particlePointerO -> landmarks;
}