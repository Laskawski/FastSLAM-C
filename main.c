#include <stdlib.h>
#include <stdio.h>
#include <cjson/cJSON.h>

#include "headers/structs.h"

int main(){
    FILE *fp;
    Particle *particles, *particlePointer;
    float *pose;
    int numParticles, len, zLen;

    numParticles = 100;

    particles = particlesInit(numParticles);

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

        fastSLAM(particles, numParticles, z, u);

        cJSON_Delete(sample);

        free(u);
        free(z);
    }

    return 0;
}