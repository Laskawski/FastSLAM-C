fastSLAM:
	gcc src/constants.c src/kalmanFilter.c src/main.c src/sampling.c src/fastSLAM.c src/landmarkInit.c src/motionModel.c src/utils.c src/particleWeight.c src/structs.c -lcjson -lblas -llapacke -o bin/fastSLAM -lm -Wextra

fastSLAM_GDB:
	gcc -g src/constants.c src/kalmanFilter.c src/main.c src/sampling.c src/fastSLAM.c src/landmarkInit.c src/motionModel.c src/utils.c src/particleWeight.c src/structs.c -lcjson -lblas -llapacke -o bin/fastSLAM -lm -Wextra

clean:
	rm bin/fastSLAM