#include "imu.h"
#include "i2c.h"
#include "rt_utils.h"
#include "config.h"
#include <stdio.h>
#include <math.h>
#include <unistd.h>

int imu_init(IMU* imu, const char* bus, pthread_mutex_t* mtx) {
    imu->fd = i2c_open(bus, MPU6050_ADDR);
    if (imu->fd < 0) { printf("[WARN] IMU not found\n"); return -1; }

    i2c_write8(imu->fd, 0x6B, 0x00, mtx);   // wake
    usleep(50000);
    i2c_write8(imu->fd, 0x1B, 0x00, mtx);   // gyro +-250 dps
    i2c_write8(imu->fd, 0x1C, 0x00, mtx);   // accel +-2 g
    i2c_write8(imu->fd, 0x1A, 0x03, mtx);   // DLPF 44 Hz

    imu->gyro_bias[0] = imu->gyro_bias[1] = 0;
    imu->fused_roll = imu->fused_pitch = 0;
    imu->last_update = time_now();
    atomic_init(&imu->roll_x1000, 0);
    atomic_init(&imu->pitch_x1000, 0);
    atomic_init(&imu->calibrated, 0);

    printf("[IMU] Initialized\n");
    return 0;
}

void imu_calibrate(IMU* imu, pthread_mutex_t* mtx) {
    printf("[IMU] Calibrating gyro...\n");
    float gx_sum = 0, gy_sum = 0;
    const int samples = 50;
    for (int i = 0; i < samples; i++) {
        int16_t gx = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x43, mtx);
        int16_t gy = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x45, mtx);
        gx_sum += gx / 131.0f;
        gy_sum += gy / 131.0f;
        usleep(10000);
    }
    imu->gyro_bias[0] = gx_sum / samples;
    imu->gyro_bias[1] = gy_sum / samples;
    atomic_store(&imu->calibrated, 1);
    printf("[IMU] Gyro bias: %.2f, %.2f\n", imu->gyro_bias[0], imu->gyro_bias[1]);
}

void imu_update(IMU* imu, pthread_mutex_t* mtx) {
    double now = time_now();
    double dt = now - imu->last_update;
    if (dt <= 0 || dt > 0.1) dt = 0.02;

    int16_t ax = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x3B, mtx);
    int16_t ay = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x3D, mtx);
    int16_t az = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x3F, mtx);
    int16_t gx = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x43, mtx);
    int16_t gy = i2c_read16_signed(imu->fd, MPU6050_ADDR, 0x45, mtx);

    float fax = ax / 16384.0f, fay = ay / 16384.0f, faz = az / 16384.0f;
    float fgx = gx / 131.0f - imu->gyro_bias[0];
    float fgy = gy / 131.0f - imu->gyro_bias[1];

    float acc_roll  = atan2f(-fax, faz) * 57.2958f;
    float acc_pitch = atan2f(fay, sqrtf(fax*fax + faz*faz)) * 57.2958f;

    imu->fused_roll  = IMU_ALPHA * (imu->fused_roll  + fgx * dt) + (1 - IMU_ALPHA) * acc_roll;
    imu->fused_pitch = IMU_ALPHA * (imu->fused_pitch + fgy * dt) + (1 - IMU_ALPHA) * acc_pitch;

    atomic_store(&imu->roll_x1000,  (int)((imu->fused_roll  - IMU_ROLL_OFFSET)  * 1000));
    atomic_store(&imu->pitch_x1000, (int)((imu->fused_pitch - IMU_PITCH_OFFSET) * 1000));
    imu->last_update = now;
}

void* thread_imu(void* arg) {
    (void)arg;
    pin_thread(CORE_SENSOR, PRIO_IMU, "IMU");
    while (!atomic_load(&G.imu.calibrated) && atomic_load(&G.running)) usleep(100000);
    int interval_ms = 1000 / IMU_SAMPLE_RATE_HZ;
    while (atomic_load(&G.running)) {
        if (G.imu.fd >= 0) imu_update(&G.imu, &G.i2c1_mtx);
        spin_ms(interval_ms);
    }
    return NULL;
}
