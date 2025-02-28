#include <stdlib.h>
#include <math.h>
#include "headers/structs.h"
#include "headers/constants.h"

int* lowVarianceSampler(Particle *particles, float weights[], int numParticles){
    int i, *indexes;
    float totalWeight, c, u, r;

    indexes = malloc(numParticles * sizeof(int));

    totalWeight = 0.0;

    for(int n = 0; n < numParticles; n++) totalWeight += weights[n];

    i = 0;
    u = 0;
    c = weights[0] / totalWeight;
    r = ((float) rand()) / RAND_MAX;

    for(int j = 0; j < numParticles; j++){
        u = (r + j) / numParticles;
        while(u > c){
            i++;
            c += weights[i] / totalWeight;
        }
        indexes[j] = i;
    }
    return indexes;
}
