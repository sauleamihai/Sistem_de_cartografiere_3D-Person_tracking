#include "i2c.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
//Linux kernel exposes each I2C bus as a character device (/dev/i2c-N) via the i2c-dev module, and you drive it through ioctl(i2c-dev.h) calls.
//  The i2c-dev module is a userspace interface to the I2C subsystem, and it allows you to perform I2C transactions from user space.
//  You can use the I2C_RDWR ioctl command to perform combined write-read transactions, which is more efficient than separate write and read calls.
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

int i2c_open(const char* bus, int addr) {
    int fd = open(bus, O_RDWR);
    if (fd < 0) return -1;
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// Single combined write-reg / read-2-bytes transaction (repeated START).
// Halves the transaction count vs separate write()+read().
uint16_t i2c_read16(int fd, uint8_t dev_addr, uint8_t reg, pthread_mutex_t* mtx) {
    struct i2c_rdwr_ioctl_data data;
    struct i2c_msg msgs[2];
    uint8_t outbuf = reg;
    uint8_t inbuf[2];

    msgs[0].addr = dev_addr;  msgs[0].flags = 0;        msgs[0].len = 1; msgs[0].buf = &outbuf;
    msgs[1].addr = dev_addr;  msgs[1].flags = I2C_M_RD; msgs[1].len = 2; msgs[1].buf = inbuf;

    data.msgs = msgs;
    data.nmsgs = 2;

    pthread_mutex_lock(mtx);
    int ret = ioctl(fd, I2C_RDWR, &data);
    pthread_mutex_unlock(mtx);

    if (ret < 0) return 0;
    return (inbuf[0] << 8) | inbuf[1];
}

int16_t i2c_read16_signed(int fd, uint8_t dev_addr, uint8_t reg, pthread_mutex_t* mtx) {
    return (int16_t)i2c_read16(fd, dev_addr, reg, mtx);
}

void i2c_write8(int fd, uint8_t reg, uint8_t val, pthread_mutex_t* mtx) {
    uint8_t buf[2] = { reg, val };
    pthread_mutex_lock(mtx);
    (void)!write(fd, buf, 2);
    pthread_mutex_unlock(mtx);
}
