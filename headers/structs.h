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

Particle* fastSLAM(Particle *particles, int numParticles, float *z, float *u);

Particle* particlesInit(int numParticles);

Particle* particlesCopy(int numParticles, Particle *particlesO, Particle *particlesD);

int* lowVarianceSampler(Particle particles[], float weights[], int numParticles);