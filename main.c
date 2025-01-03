#include <stdlib.h>
#include <stdio.h>
#include <cjson/cJSON.h>

#include "headers/structs.h"

int main(){
    FILE *fp;
    Particle *particles, *particlesAux, *particlePointer;
    float *pose;
    int numParticles, len, zLen;

    numParticles = 100;
    particlesAux = malloc(numParticles * sizeof(Particle));

    for(int n = 0; n < numParticles; n++){
        pose = malloc(3 * sizeof(float));
        pose[0] = 0.0;
        pose[1] = 0.0;
        pose[2] = 0.0;

        particlePointer = &particlesAux[n];
        particlePointer -> weight = 0.0;
        particlePointer -> pose = pose;
        particlePointer -> mapSize = 0;
        particlePointer -> landmarks = malloc(sizeof(Landmark));
    }

    for(int i = 0; i < 1000; i++){
        cJSON *sample, *speed, *steer, *cam_cones, *blue, *yellow, *conePos, *coord;
        char adress[92], buffer[1024];
        float *u, *z;

        sprintf(adress, "/home/guilherme/Documents/Iniciação_Científica/tracks/data_skidpad/sample_%d.json", i);

        fp = fopen(adress, "r");
        len = fread(buffer, 1, sizeof(buffer), fp);  
        fclose(fp);

        sample = cJSON_Parse(buffer);

        speed = cJSON_GetObjectItemCaseSensitive(sample, "speed");
        steer = cJSON_GetObjectItemCaseSensitive(sample, "steer");

        cam_cones = cJSON_GetObjectItemCaseSensitive(sample, "cam_cones");
        blue = cJSON_GetObjectItemCaseSensitive(cam_cones, "blue");
        yellow = cJSON_GetObjectItemCaseSensitive(cam_cones, "yellow");

        zLen = cJSON_GetArraySize(blue) + cJSON_GetArraySize(yellow);

        u = malloc(2 * sizeof(float));
        z = malloc(zLen * 2 * sizeof(float));

        u[0] = speed -> valuedouble;
        u[1] = steer -> valuedouble;

        int j = 0;

        cJSON_ArrayForEach(conePos, blue){
            cJSON_ArrayForEach(coord, conePos){
                z[j] = coord -> valuedouble;
                j++;
            }
        }

        cJSON_ArrayForEach(conePos, yellow){
            cJSON_ArrayForEach(coord, conePos){
                z[j] = coord -> valuedouble;
                j++;
            }
        }

        particles = fastSLAM(particlesAux, numParticles, z, u);
        free(particlesAux);
        particlesAux = particles;

        cJSON_Delete(sample);
        cJSON_Delete(speed);
        cJSON_Delete(steer);
        cJSON_Delete(cam_cones);
        cJSON_Delete(blue);
        cJSON_Delete(yellow);

        free(u);
        free(z);
    }

    return 0;
}