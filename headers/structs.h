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

void fastSLAM(Particle *particles, int numParticles, float *z, float *u);

void landmarksCopy(Landmark *landmarkPointerO, Landmark *landmarkPointerD);

Particle* particlesInit(int numParticles);

void freeParticles(Particle *particles, int numParticles);

void particlesCopy(Particle *particlePointerO, Particle *particlePointerD);

int* lowVarianceSampler(Particle particles[], float weights[], int numParticles);