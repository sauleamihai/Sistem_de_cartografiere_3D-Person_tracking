#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include <netinet/in.h>
#include "config.h"

// ============================================================================
// types.h - Shared data structures and the single global state object.
//
// Inter-thread communication strategy:
//   * Fast, single-value state (angles, positions, flags) lives in _Atomic
//     fields -> lock-free reads/writes, no mutex on the RT path.
//   * Shared *buses* (I2C) and the network client struct are protected by
//     mutexes, because they are multi-step transactions, not single words.
// ============================================================================

typedef struct {
    int fd;
    uint16_t zero_raw;
    float scale_factor;
    float offset;
    int inverted;

    float buffer[ENCODER_FILTER_SIZE];   // moving-average ring buffer
    int   buf_idx;

    _Atomic float    angle;   // last filtered angle (lock-free read)
    _Atomic uint16_t raw;
} Encoder;

typedef struct {
    int fd;
    float gyro_bias[2];
    float fused_roll;
    float fused_pitch;
    double last_update;
    _Atomic int roll_x1000;   // fixed-point so it fits an atomic int
    _Atomic int pitch_x1000;
    _Atomic int calibrated;
} IMU;

typedef struct {
    _Atomic int pos;            // current step position
    _Atomic int target;         // target step position
    _Atomic int moving;

    double speed;               // owned by motor control loop only
    double last_step_time;

    double max_speed;
    double cruise_speed;
    double accel;
    double min_speed;

    Encoder* encoder;
    _Atomic float target_angle; // written by scanner/net, read by motor loop
    double last_correction;     // owned by motor control loop only
} Motor;

struct __attribute__((packed)) Point {
    float x, y, z;
    uint8_t r, g, b;
};

typedef struct {
    Motor roll;
    Motor pitch;
    Encoder enc_roll;
    Encoder enc_pitch;
    IMU imu;

    _Atomic int running;
    _Atomic int emergency;
    _Atomic int scanning;
    _Atomic int scan_mode;   // 0 = pitch only, 1 = grid (pitch + roll)
    _Atomic int calibrated;

    double temp1, temp2;
    int fan_speed;

    struct sockaddr_in client;
    _Atomic int client_set;

    pthread_mutex_t i2c0_mtx;
    pthread_mutex_t i2c1_mtx;
    pthread_mutex_t net_mtx;

    int gpio_handle;         // shared lgpio handle, opened by motor loop
} GlobalState;

extern GlobalState G;

#endif // TYPES_H
