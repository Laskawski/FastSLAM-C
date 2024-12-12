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

void landmarksCopy(Landmark *landmarkPointerO, Landmark *landmarkPointerD){
    cblas_scopy(2, landmarkPointerO -> mean, 1, landmarkPointerD -> mean, 1);
    cblas_scopy(4, landmarkPointerO -> covariance, 1, landmarkPointerD -> covariance, 1);
    landmarkPointerD -> counter = landmarkPointerO -> counter;
}

Particle* particlesInit(int numParticles){
    Landmark *landmarks, *landmarkPointer;
    Particle *particles, *particlePointer;
    float *pose, *mean, *covariance;

    particles = malloc(numParticles * sizeof(Particle));

    for(int n = 0; n < numParticles; n++){
        pose = malloc(3 * sizeof(float));
        landmarks = malloc(100 * sizeof(Landmark));
        
        for(int i = 0; i < 100; i++){
            mean = malloc(2 * sizeof(float));
            covariance = malloc(4 * sizeof(float));

            landmarkPointer = &landmarks[i];
            landmarkPointer -> mean = mean;
            landmarkPointer -> covariance = covariance;
            landmarkPointer -> counter = 0;
        }

        particlePointer = &particles[n];
        particlePointer -> weight = 0.0;
        particlePointer -> pose = pose;
        particlePointer -> mapSize = 0;
        particlePointer -> landmarks = landmarks;
    }

    return particles;
}

void freeParticles(Particle *particles, int numParticles){
    Landmark *landmarksPointer, *landmarkPointer;
    Particle *particlePointer;

    for(int i = 0; i < numParticles; i++){
        particlePointer = &particles[i];
        landmarksPointer = particlePointer -> landmarks;

        for(int j = 0; j < 100; j++){
            landmarkPointer = &landmarksPointer[j];
            free(landmarkPointer -> mean);
            free(landmarkPointer -> covariance);
        }
        free(landmarksPointer);
        free(particlePointer -> pose);
    }
    free(particles);
}

void particlesCopy(Particle *particlePointerO, Particle *particlePointerD){
    Landmark *landmarksPointerO, *landmarksPointerD;

    particlePointerD -> weight = particlePointerO -> weight;
    particlePointerD -> mapSize = particlePointerO -> mapSize;
    cblas_scopy(3, particlePointerO -> pose, 1, particlePointerD -> pose, 1);

    landmarksPointerO = particlePointerO -> landmarks;
    landmarksPointerD = particlePointerD -> landmarks;

    for(int n = 0; n < particlePointerO -> mapSize; n++){
        landmarksCopy(&landmarksPointerO[n], &landmarksPointerD[n]);
    }
}