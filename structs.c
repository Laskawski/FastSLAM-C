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

Particle* particlesInit(int numParticles){
    Particle *particles, *particlePointer;
    float *pose;

    particles = malloc(numParticles * sizeof(Particle));

    for(int n = 0; n < numParticles; n++){
        pose = malloc(3 * sizeof(float));
        pose[0] = 0.0;
        pose[1] = 0.0;
        pose[2] = 0.0;

        particlePointer = &particles[n];
        particlePointer -> weight = 0.0;
        particlePointer -> pose = pose;
        particlePointer -> mapSize = 0;
        particlePointer -> landmarks = malloc(sizeof(Landmark));
    }

    return particles;
}

Particle* particlesCopy(int numParticles, Particle *particlesO, Particle *particlesD){
    Particle *particlesPointerO, *particlesPointerD;

    for(int n = 0; n < numParticles; n++){
        particlesPointerO = &particlesO[n];
        particlesPointerD = &particlesD[n];
        particlesPointerD -> weight = particlesPointerO -> weight;
        particlesPointerD -> pose = particlesPointerO -> pose;
        particlesPointerD -> mapSize = particlesPointerO -> mapSize;
        particlesPointerD -> landmarks = particlesPointerO -> landmarks;
    }
}