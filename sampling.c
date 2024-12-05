#include <stdlib.h>
#include <math.h>
#include "headers/structs.h"
#include "headers/constants.h"

int* lowVarianceSampler(Particle *particles, float weights[], int numParticles){
    int c, i, *indexes;
    float u, r;

    indexes = malloc(numParticles * sizeof(int));

    i = 0;
    u = 0;
    c = weights[0];
    r = ((float) rand()) / RAND_MAX;

    for(int j = 0; j < numParticles; j++){
        u = (r + j) / numParticles;
        while(u > c){
            i++;
            c += weights[i];
        }
        indexes[j] = i;
    }
    return indexes;
}

float* standardNormalDist(){
    float u1, u2, r, theta;
    float *samples = malloc(2 * sizeof(float));

    u1 = ((float) rand()) / RAND_MAX;
    u2 = ((float) rand()) / RAND_MAX;

    while (u1==0 || u2==0){
        u1 = ((float) rand()) / RAND_MAX;
        u2 = ((float) rand()) / RAND_MAX;
    }

    r = sqrt(-2 * log(u1));
    theta = 2 * M_PI * u2;
    samples[0] = r * cos(theta);
    samples[1] = r * sin(theta);

    return samples;
}
