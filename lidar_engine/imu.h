#ifndef IMU_H
#define IMU_H
#include "types.h"

int  imu_init(IMU* imu, const char* bus, pthread_mutex_t* mtx);
void imu_calibrate(IMU* imu, pthread_mutex_t* mtx);
void imu_update(IMU* imu, pthread_mutex_t* mtx);  // complementary filter step

#endif

void* thread_imu(void* arg);  // low-rate complementary-filter thread
