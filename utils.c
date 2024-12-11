#include <math.h>
#include <stdlib.h>

float* rotationMatrix(float angle){
    float sine, cosine;
    float* rotMat = malloc(4 * sizeof(float));

    sine = sin(angle);
    cosine = cos(angle);
    rotMat[0] = cosine;
    rotMat[1] = sine;
    rotMat[2] = -sine;
    rotMat[3] = cosine;

    return rotMat;
}

float* identityMatrix(int numRows){
    float* matrix = malloc(numRows * numRows * sizeof(float));
    int n = 0;

    for(int i = 0; i < numRows; i++){
        for(int j = 0; j < numRows; j++){
            if(i == j) matrix[n] = 1.0;
            else matrix[n] = 0.0;
            n++;
        }
    }

    return matrix;
}

float argMax(float *vector, int numElements){
    float max;
    int argMax;

    max = vector[0];
    argMax = 0;

    for(int i = 0; i < numElements; i++){
        if (vector[i] > max){
            max = vector[i];
            argMax = i;
        }
    }

    return argMax;
}