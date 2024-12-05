void correct(float* mean, float* covariance, float* z, float* pose);

float* kalmanGain(float *mean, float *covariance, float *pose, float *rotMat);

float* measurementCovariance(float *mean, float *covariance, float *pose, float *rotMat);

float* predMeasurement(float *mean, float *pose, float *rotMat);