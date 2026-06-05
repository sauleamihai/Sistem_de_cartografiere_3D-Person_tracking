#include "encoder.h"
#include "i2c.h"
#include "config.h"
#include <stdio.h>
#include <unistd.h>

int encoder_init(Encoder* e, const char* bus, float scale, float offset,
                 int inv, pthread_mutex_t* mtx) {
    //open i2c bus and read zero position
    //two diffrent i2c buses are used to avoid interference
    e->fd = i2c_open(bus, AS5600_ADDR);
    if (e->fd < 0) { printf("[ERR] Encoder open failed: %s\n", bus); return -1; }
    //config parameters from config file
    e->scale_factor = scale;
    e->offset = offset;
    e->inverted = inv;
    e->buf_idx = 0;
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) e->buffer[i] = 0;
    //atomics for thread-safe access without mutex during reads
    atomic_init(&e->angle, 0);
    atomic_init(&e->raw, 0);
    //config from AS5600 datasheet recommends a delay after power-up before reading
    usleep(50000);
    //we set the zero position on startup, but it can be reset later with encoder_reset_zero()
    //this is a relative reference scheme, not absolute. The device assumes it boots at mechanical zero
    e->zero_raw = i2c_read16(e->fd, AS5600_ADDR, AS5600_ANGLE_REG, mtx);

    printf("[ENC] %s: zero=%d scale=%.4f off=%.2f inv=%d\n",
           bus, e->zero_raw, scale, offset, inv);
    return 0;
}

float encoder_update(Encoder* e, pthread_mutex_t* mtx) {
    //One combined I2C transaction (write register pointer, repeated-START, read 2 bytes). 
    // The two bytes assemble into a 12-bit value (the top 4 bits are zero). 
    // Raw is cached atomically in case anything wants the unprocessed count
    uint16_t raw = i2c_read16(e->fd, AS5600_ADDR, AS5600_ANGLE_REG, mtx);
    atomic_store(&e->raw, raw);

    //4096 = 2¹², so this maps the 0–4095\
    //count linearly onto 0–360°. curr is the absolute angle, zero is the boot reference in the same units, and rel is the displacement from home.
    float curr = (raw * 360.0f) / 4096.0f;
    float zero = (e->zero_raw * 360.0f) / 4096.0f;
    float rel = curr - zero;
    //Normalizing into [−180, 180] gives the shortest signed angular distance, which is what you physically want.
    while (rel > 180.0f)  rel -= 360.0f;
    while (rel < -180.0f) rel += 360.0f;
    //Apply inversion, scaling, and offset to get the final angle in degrees. The filter is a simple moving average, which is good enough for this 
    // application and very cheap to compute.
    if (e->inverted) rel = -rel;

    float cal = rel * e->scale_factor + e->offset;
    //8-tap moving average filter
    //it adds group delay of roughly (N−1)/2 = 3.5 samples of lag. At the 250 Hz moving rate that's ~14 ms of latency between true and reported angle
    //This is a tradeoff to reduce noise and jitter in the angle readings, which can cause instability in the control loop.
    // The filter size can be adjusted in config.h.
    e->buffer[e->buf_idx] = cal;
    e->buf_idx = (e->buf_idx + 1) % ENCODER_FILTER_SIZE;
    
    float sum = 0;
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) sum += e->buffer[i];
    float filtered = sum / ENCODER_FILTER_SIZE;
    //The filtered angle is stored atomically for lock-free access by the control loop. This is the main output of the encoder module.
    atomic_store(&e->angle, filtered);
    return filtered;
}
//A lock-free read of the last filtered value.
// This allows the control loop to read the encoder angle without blocking on the mutex, which is important for real-time performance. 
// The angle may be slightly stale (up to one polling interval), but that's usually acceptable in a control system.
float encoder_get(Encoder* e) { return atomic_load(&e->angle); }

void encoder_reset_zero(Encoder* e, pthread_mutex_t* mtx) {
    //Recapture the current raw angle as the new zero reference. 
    // This allows the user to re-zero the system at any position, which can be useful for calibration 
    // or if the system is moved while powered off.
    e->zero_raw = i2c_read16(e->fd, AS5600_ADDR, AS5600_ANGLE_REG, mtx);
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) e->buffer[i] = 0;
    atomic_store(&e->angle, 0);
}

#include "rt_utils.h"

// Polls both encoders. Fast when either axis is moving, slow when idle, to
// keep the shared I2C buses quiet during capture windows.
void* thread_encoder(void* arg) {
    (void)arg;
    pin_thread(CORE_SENSOR, PRIO_ENCODER, "Encoder");
    while (atomic_load(&G.running)) {
        encoder_update(&G.enc_roll,  &G.i2c0_mtx);
        encoder_update(&G.enc_pitch, &G.i2c1_mtx);
        int moving = atomic_load(&G.roll.moving) || atomic_load(&G.pitch.moving);
        spin_ms(moving ? ENCODER_POLL_FAST_MS : ENCODER_POLL_SLOW_MS);
    }
    return NULL;
}
