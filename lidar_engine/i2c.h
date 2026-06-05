#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <pthread.h>

// Thin I2C transport layer.
//
// BUGFIX vs v19: read16 now takes an explicit device address. The old version
// hardcoded AS5600_ADDR inside the I2C_RDWR messages, so every IMU read was
// actually addressed to the encoder (0x36) instead of the MPU6050 (0x68).
// I2C_RDWR uses the per-message address and ignores the I2C_SLAVE setting,
// which is why init (write-based) worked but reads returned garbage.

int      i2c_open(const char* bus, int addr);
uint16_t i2c_read16(int fd, uint8_t dev_addr, uint8_t reg, pthread_mutex_t* mtx);
int16_t  i2c_read16_signed(int fd, uint8_t dev_addr, uint8_t reg, pthread_mutex_t* mtx);
void     i2c_write8(int fd, uint8_t reg, uint8_t val, pthread_mutex_t* mtx);

#endif
