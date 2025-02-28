fastSLAM:
	gcc src/constants.c src/kalmanFilter.c src/main.c src/sampling.c src/fastSLAM.c src/landmarkInit.c src/motionModel.c src/utils.c src/particleWeight.c src/structs.c -lcjson -lblas -llapacke -o bin/fastSLAM -lm -Wextra

fastSLAM_GDB:
	gcc -g src/constants.c src/kalmanFilter.c src/main.c src/sampling.c src/fastSLAM.c src/landmarkInit.c src/motionModel.c src/utils.c src/particleWeight.c src/structs.c -lcjson -lblas -llapacke -o bin/fastSLAM -lm -Wextra

library:
	gcc -c src/constants.c -o bin/constants.o
	gcc -c src/kalmanFilter.c -o bin/kalmanFilter.o
	gcc -c src/sampling.c -o bin/sampling.o
	gcc -c src/fastSLAM.c -o bin/fastSLAM.o
	gcc -c src/landmarkInit.c -o bin/landmarkInit.o
	gcc -c src/motionModel.c -o bin/motionModel.o
	gcc -c src/utils.c -o bin/utils.o
	gcc -c src/particleWeight.c -o bin/particleWeight.o
	gcc -c src/structs.c -o bin/structs.o

	ar rcs fastSLAM.a bin/constants.o bin/kalmanFilter.o bin/sampling.o bin/fastSLAM.o bin/landmarkInit.o bin/motionModel.o bin/utils.o bin/particleWeight.o bin/structs.o

clean:
	rm bin/fastSLAM