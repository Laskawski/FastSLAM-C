fastSLAM:
	gcc constants.c kalmanFilter.c main.c sampling.c fastSLAM.c landmarkInit.c motionModel.c utils.c particleWeight.c structs.c -lcjson -lblas -llapacke -o fastSLAM -lm -Wextra

fastSLAM_GDB:
	gcc -g constants.c kalmanFilter.c main.c sampling.c fastSLAM.c landmarkInit.c motionModel.c utils.c particleWeight.c structs.c -lcjson -lblas -llapacke -o fastSLAM -lm -Wextra

clean:
	rm fastSLAM