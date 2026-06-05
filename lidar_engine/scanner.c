#define _GNU_SOURCE
#include "scanner.h"
#include "types.h"
#include "config.h"
#include "lidar.h"
#include "motor.h"
#include "encoder.h"
#include "rt_utils.h"
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>

// Move an axis to `angle`, wait until the step target is reached (bounded),
// let vibration settle, and return the *measured* encoder angle.
static float goto_and_settle(Motor* m, float angle, int settle_ms) {
    motor_set_target(m, angle);
    double t0 = time_now();
    while (atomic_load(&m->pos) != atomic_load(&m->target) && (time_now() - t0) < 3.0)
        spin_us(100);
    spin_ms(settle_ms);
    atomic_store(&m->moving, 0);
    return encoder_get(m->encoder);
}

// Sweep one axis across [lo, hi] capturing at each step.
static void sweep_axis(int uart, int sock, ScanAxis axis,
                       Motor* m, float lo, float hi, int settle_ms) {
    for (float a = lo; a <= hi && atomic_load(&G.scanning); a += SCAN_STEP) {
        if (atomic_load(&G.emergency)) break;
        float actual = goto_and_settle(m, a, settle_ms);
        lidar_capture_angle(uart, sock, axis, actual);
    }
}

void* thread_scanner(void* arg) {
    (void)arg;
    pin_thread(CORE_SCAN, PRIO_SCAN, "Scanner");

    while (!atomic_load(&G.calibrated) && atomic_load(&G.running)) usleep(100000);

    int uart = lidar_open_uart();
    if (uart < 0) { printf("[ERR] UART open failed\n"); return NULL; }
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    printf("[SCAN] Ready\n");

    while (atomic_load(&G.running)) {
        if (!atomic_load(&G.scanning)) { usleep(50000); continue; }

        // Pitch sweep (always)
        sweep_axis(uart, sock, AXIS_PITCH, &G.pitch,
                   PITCH_MIN, PITCH_MAX, SCAN_SETTLE_PITCH_MS);

        // Roll sweep (grid mode only) - recenter pitch first
        if (atomic_load(&G.scan_mode) == 1 && atomic_load(&G.scanning)) {
            goto_and_settle(&G.pitch, 0, SCAN_SETTLE_PITCH_MS);
            sweep_axis(uart, sock, AXIS_ROLL, &G.roll,
                       ROLL_MIN, ROLL_MAX, SCAN_SETTLE_ROLL_MS);
            motor_set_target(&G.roll, 0);
        }

        atomic_store(&G.scanning, 0);
        motor_set_target(&G.pitch, 0);

        pthread_mutex_lock(&G.net_mtx);
        if (atomic_load(&G.client_set))
            sendto(sock, "STATUS:DONE", 11, 0,
                   (struct sockaddr*)&G.client, sizeof(G.client));
        pthread_mutex_unlock(&G.net_mtx);
    }

    close(uart);
    close(sock);
    return NULL;
}
