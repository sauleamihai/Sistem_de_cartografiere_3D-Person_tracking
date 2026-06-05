#define _GNU_SOURCE
#include "types.h"
#include "config.h"
#include "rt_utils.h"
#include "mathlut.h"
#include "encoder.h"
#include "imu.h"
#include "motor.h"
#include "scanner.h"
#include "thermal.h"
#include "network.h"
#include "watchdog.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sys/mman.h>

// BUGFIX vs v19: clean shutdown on SIGINT/SIGTERM. Previously a systemd stop
// (SIGTERM) killed the process with the watchdog still armed and GPIO/I2C
// left open. Now the handler just flips the atomic running flag; the threads
// unwind, the watchdog disarms, and main() joins + closes everything.
static void on_signal(int sig) {
    (void)sig;
    atomic_store(&G.running, 0);
}

int main(void) {
    memset(&G, 0, sizeof(G));
    atomic_init(&G.running, 1);
    atomic_init(&G.emergency, 0);
    atomic_init(&G.scanning, 0);
    atomic_init(&G.calibrated, 0);
    atomic_init(&G.client_set, 0);

    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    pthread_mutex_init(&G.i2c0_mtx, NULL);
    pthread_mutex_init(&G.i2c1_mtx, NULL);
    pthread_mutex_init(&G.net_mtx,  NULL);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) perror("mlockall");

    init_lut();

    printf("[CAL] Starting calibration...\n");
    if (encoder_init(&G.enc_roll, I2C_BUS_0,
                     ROLL_SCALE_FACTOR, ROLL_OFFSET, ROLL_INVERTED, &G.i2c0_mtx) < 0) {
        printf("[FATAL] Roll encoder init failed\n"); return 1;
    }
    if (encoder_init(&G.enc_pitch, I2C_BUS_1,
                     PITCH_SCALE_FACTOR, PITCH_OFFSET, PITCH_INVERTED, &G.i2c1_mtx) < 0) {
        printf("[FATAL] Pitch encoder init failed\n"); return 1;
    }
    if (imu_init(&G.imu, I2C_BUS_1, &G.i2c1_mtx) == 0) {
        sleep(1);
        imu_calibrate(&G.imu, &G.i2c1_mtx);
    }

    motor_init(&G.roll,  &G.enc_roll,
               ROLL_MAX_SPEED, ROLL_CRUISE_SPEED, ROLL_ACCELERATION, ROLL_MIN_SPEED);
    motor_init(&G.pitch, &G.enc_pitch,
               PITCH_MAX_SPEED, PITCH_CRUISE_SPEED, PITCH_ACCELERATION, PITCH_MIN_SPEED);

    atomic_store(&G.calibrated, 1);
    printf("[CAL] Complete\n\n");

    pthread_t t_enc, t_imu, t_scan, t_temp, t_net, t_wd;
    pthread_create(&t_enc,  NULL, thread_encoder,  NULL);
    pthread_create(&t_imu,  NULL, thread_imu,      NULL);
    pthread_create(&t_scan, NULL, thread_scanner,  NULL);
    pthread_create(&t_temp, NULL, thread_temp,     NULL);
    pthread_create(&t_net,  NULL, thread_net,      NULL);
    pthread_create(&t_wd,   NULL, thread_watchdog, NULL);

    usleep(200000);

    motor_control_loop();          // runs on main thread (core 3) until running=0

    atomic_store(&G.running, 0);
    pthread_join(t_enc,  NULL);
    pthread_join(t_imu,  NULL);
    pthread_join(t_scan, NULL);
    pthread_join(t_temp, NULL);
    pthread_join(t_net,  NULL);
    pthread_join(t_wd,   NULL);

    if (G.enc_roll.fd  >= 0) close(G.enc_roll.fd);
    if (G.enc_pitch.fd >= 0) close(G.enc_pitch.fd);
    if (G.imu.fd       >= 0) close(G.imu.fd);

    printf("\n[EXIT] Shutdown complete\n");
    return 0;
}
