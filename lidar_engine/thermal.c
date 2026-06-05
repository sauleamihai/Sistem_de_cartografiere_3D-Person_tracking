#define _GNU_SOURCE
#include "thermal.h"
#include "types.h"
#include "config.h"
#include "i2c.h"
#include "rt_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <lgpio.h>

// ----- BMP280 temperature compensation -----
// BUGFIX vs v19: the old read_bmp280 divided the raw ADC by a constant and
// returned that as "temperature", ignoring the factory trim. Now we read the
// dig_T1..T3 calibration words once and apply the datasheet formula.
typedef struct { uint16_t t1; int16_t t2, t3; int have; } BmpCalib;

static uint16_t bmp_read16le(int fd, uint8_t reg, pthread_mutex_t* mtx) {
    // calibration regs are little-endian: LSB at reg, MSB at reg+1
    uint8_t lo, hi;
    uint8_t r = reg;
    pthread_mutex_lock(mtx);
    (void)!write(fd, &r, 1); (void)!read(fd, &lo, 1);
    r = reg + 1;
    (void)!write(fd, &r, 1); (void)!read(fd, &hi, 1);
    pthread_mutex_unlock(mtx);
    return (uint16_t)((hi << 8) | lo);
}

static void bmp280_read_calib(int fd, BmpCalib* c, pthread_mutex_t* mtx) {
    c->t1 = bmp_read16le(fd, 0x88, mtx);
    c->t2 = (int16_t)bmp_read16le(fd, 0x8A, mtx);
    c->t3 = (int16_t)bmp_read16le(fd, 0x8C, mtx);
    c->have = (c->t1 != 0);
}

static double bmp280_temp_c(int fd, BmpCalib* c, pthread_mutex_t* mtx) {
    pthread_mutex_lock(mtx);
    uint8_t reg = 0xFA;
    uint8_t buf[3];
    (void)!write(fd, &reg, 1);
    int n = read(fd, buf, 3);
    pthread_mutex_unlock(mtx);
    if (n != 3 || !c->have) return NAN;

    int32_t adc = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    double var1 = (((double)adc) / 16384.0 - ((double)c->t1) / 1024.0) * (double)c->t2;
    double var2 = ((((double)adc) / 131072.0 - ((double)c->t1) / 8192.0) *
                   (((double)adc) / 131072.0 - ((double)c->t1) / 8192.0)) * (double)c->t3;
    return (var1 + var2) / 5120.0;
}

static double read_htu21d(int fd, pthread_mutex_t* mtx) {
    pthread_mutex_lock(mtx);
    uint8_t cmd = 0xF3;          // no-hold temperature measurement
    (void)!write(fd, &cmd, 1);
    usleep(55000);
    uint8_t buf[3];
    int n = read(fd, buf, 3);
    pthread_mutex_unlock(mtx);
    if (n != 3) return NAN;
    return -46.85 + 175.72 * (((buf[0]<<8)|buf[1]) & 0xFFFC) / 65536.0;
}

void* thread_temp(void* arg) {
    (void)arg;
    pin_thread(CORE_NONRT, PRIO_NONRT, "Temp");

    int h = lgGpiochipOpen(MOTOR_CHIP);
    if (h < 0) return NULL;
    int fan_pins[] = { FAN1_PWM, FAN1_IN1, FAN1_IN2, FAN2_PWM, FAN2_IN3, FAN2_IN4 };
    for (unsigned k = 0; k < sizeof(fan_pins)/sizeof(fan_pins[0]); k++)
        lgGpioClaimOutput(h, 0, fan_pins[k], 0);
    lgGpioWrite(h, FAN1_IN1, 1); lgGpioWrite(h, FAN1_IN2, 0);
    lgGpioWrite(h, FAN2_IN3, 1); lgGpioWrite(h, FAN2_IN4, 0);

    int htu = i2c_open(I2C_BUS_0, HTU21D_ADDR);
    int bmp = i2c_open(I2C_BUS_0, BMP280_ADDR);
    BmpCalib calib = {0};

    if (htu >= 0) {
        pthread_mutex_lock(&G.i2c0_mtx);
        uint8_t r = 0xFE; (void)!write(htu, &r, 1);   // soft reset
        pthread_mutex_unlock(&G.i2c0_mtx);
        usleep(15000);
    }
    if (bmp >= 0) {
        i2c_write8(bmp, 0xF4, 0x27, &G.i2c0_mtx);      // normal mode, osrs x1
        usleep(10000);
        bmp280_read_calib(bmp, &calib, &G.i2c0_mtx);
    }
    printf("[TEMP] HTU21D=%s, BMP280=%s\n",
           htu >= 0 ? "OK" : "FAIL", bmp >= 0 ? "OK" : "FAIL");

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    double last_telem = 0;

    while (atomic_load(&G.running)) {
        double t1 = (htu >= 0) ? read_htu21d(htu, &G.i2c0_mtx) : NAN;
        double t2 = (bmp >= 0) ? bmp280_temp_c(bmp, &calib, &G.i2c0_mtx) : NAN;
        if (isnan(t1)) t1 = TEMP_MIN;
        if (isnan(t2)) t2 = TEMP_MIN;
        G.temp1 = t1; G.temp2 = t2;
        double max_t = (t1 > t2) ? t1 : t2;

        if (max_t >= TEMP_CRITICAL) {
            atomic_store(&G.emergency, 1);
            atomic_store(&G.scanning, 0);
            lgTxPwm(h, FAN1_PWM, 1000, 100, 0, 0);
            lgTxPwm(h, FAN2_PWM, 1000, 100, 0, 0);
            printf("[CRIT] Temperature: %.1f C\n", max_t);
            pthread_mutex_lock(&G.net_mtx);
            if (atomic_load(&G.client_set)) {
                char msg[64];
                snprintf(msg, sizeof(msg), "CRITICAL_TEMP:%.1f", max_t);
                sendto(sock, msg, strlen(msg), 0,
                       (struct sockaddr*)&G.client, sizeof(G.client));
            }
            pthread_mutex_unlock(&G.net_mtx);
            break;
        }

        int fan = 0;
        if (max_t > TEMP_MIN) {
            fan = (int)(((max_t - TEMP_MIN) / (TEMP_MAX - TEMP_MIN)) * 100);
            if (fan > 100) fan = 100;
        }
        lgTxPwm(h, FAN1_PWM, 1000, (float)fan, 0, 0);
        lgTxPwm(h, FAN2_PWM, 1000, (float)fan, 0, 0);
        G.fan_speed = fan;

        if (time_now() - last_telem >= 5.0) {
            char msg[64];
            snprintf(msg, sizeof(msg), "TEMP:%.1f,%.1f,%d,%d", t1, t2, fan, fan);
            pthread_mutex_lock(&G.net_mtx);
            if (atomic_load(&G.client_set))
                sendto(sock, msg, strlen(msg), 0,
                       (struct sockaddr*)&G.client, sizeof(G.client));
            pthread_mutex_unlock(&G.net_mtx);
            last_telem = time_now();
        }
        sleep(2);
    }

    close(sock);
    lgGpiochipClose(h);
    if (htu >= 0) close(htu);
    if (bmp >= 0) close(bmp);
    return NULL;
}
