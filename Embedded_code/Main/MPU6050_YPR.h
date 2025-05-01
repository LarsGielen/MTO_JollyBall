#ifndef MPU6050_PID_YPR
#define MPU6050_PID_YPR

// OFFSETS on flat surface

#define OFFSET_X_ACCEL -2894
#define OFFSET_Y_ACCEL -869
#define OFFSET_Z_ACCEL 1620
#define OFFSET_X_GYRO 62
#define OFFSET_Y_GYRO 6
#define OFFSET_Z_GYRO 4


void setup_MPU6050(bool calibrate);
bool readout_mpu_ypr(float* ypr);

#endif
