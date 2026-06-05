#define _GNU_SOURCE
#include "motor.h"
#include "encoder.h"
#include "rt_utils.h"
#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <lgpio.h>

void motor_init(Motor* m, Encoder* enc, double max_spd, double cruise,
                double accel, double min_spd) {
    atomic_init(&m->pos, 0);
    atomic_init(&m->target, 0);
    atomic_init(&m->moving, 0);
    atomic_init(&m->target_angle, 0.0f);
    m->speed = 0;
    m->last_step_time = time_now();
    m->max_speed = max_spd;
    m->cruise_speed = cruise;
    m->accel = accel;
    m->min_speed = min_spd;
    m->encoder = enc;
    m->last_correction = 0;
}

void motor_set_target(Motor* m, float angle) {
    atomic_store(&m->target_angle, angle);
    atomic_store(&m->target, (int)(angle * STEPS_PER_DEG));
    atomic_store(&m->moving, 1);
}

// One trapezoidal-profile step toward target. Returns immediately if the
// step interval has not elapsed. Updates m->pos by one microstep.
static void motor_step_toward(Motor* m, int step_pin, int dir_pin, double now) {
    int pos = atomic_load(&m->pos);
    int tgt = atomic_load(&m->target);
    if (pos == tgt) { m->speed = 0; return; }

    double interval = (fabs(m->speed) < m->min_speed)
                    ? (1.0 / m->min_speed) : (1.0 / fabs(m->speed));
    if ((now - m->last_step_time) < interval) return;

    int dist = tgt - pos;
    double dt = now - m->last_step_time;
    if (dt > 0.05) dt = 0.05;

    double stop_dist = (m->speed * m->speed) / (2.0 * m->accel);
    if (abs(dist) <= (int)(stop_dist + 1)) {            // decelerate
        if (m->speed > 0) m->speed = fmax(m->min_speed,  m->speed - m->accel * dt);
        else              m->speed = fmin(-m->min_speed, m->speed + m->accel * dt);
    } else {                                            // accelerate to cruise
        if (dist > 0) m->speed = fmin(m->cruise_speed,  m->speed + m->accel * dt);
        else          m->speed = fmax(-m->cruise_speed, m->speed - m->accel * dt);
    }

    lgGpioWrite(G.gpio_handle, dir_pin, m->speed > 0 ? 1 : 0);
    lgGpioWrite(G.gpio_handle, step_pin, 1);
    spin_us(10);
    lgGpioWrite(G.gpio_handle, step_pin, 0);

    atomic_store(&m->pos, pos + (m->speed > 0 ? 1 : -1));
    m->last_step_time = now;
}

// Gentle encoder-based correction, only while the axis is idle. Capped to a
// few steps with a cooldown so it never fights the profile or oscillates.
// NOTE: still blocks the loop for up to MAX_CORRECTION_STEPS*CORRECTION_PULSE_US.
// Left as-is intentionally (it only runs when both axes are idle), but this is
// the spot to convert to one-step-per-iteration if you ever correct while moving.
static void motor_correct(Motor* m, int step_pin, int dir_pin, float tolerance) {
    if (!m->encoder) return;
    double now = time_now();
    if ((now - m->last_correction) * 1000 < CORRECTION_COOLDOWN_MS) return;

    float actual = encoder_get(m->encoder);
    float error  = atomic_load(&m->target_angle) - actual;
    if (fabsf(error) < tolerance) return;

    m->last_correction = now;
    int steps = (int)(error * STEPS_PER_DEG);
    if (abs(steps) > MAX_CORRECTION_STEPS)
        steps = (steps > 0) ? MAX_CORRECTION_STEPS : -MAX_CORRECTION_STEPS;
    if (steps == 0) return;

    lgGpioWrite(G.gpio_handle, dir_pin, steps > 0 ? 1 : 0);
    for (int i = 0; i < abs(steps); i++) {
        lgGpioWrite(G.gpio_handle, step_pin, 1);
        spin_us(10);
        lgGpioWrite(G.gpio_handle, step_pin, 0);
        spin_us(CORRECTION_PULSE_US);
    }
    atomic_fetch_add(&m->pos, steps);
}

void motor_control_loop(void) {
    pin_thread(CORE_MOTOR, PRIO_MOTOR, "MotorCtrl");

    G.gpio_handle = lgGpiochipOpen(MOTOR_CHIP);
    if (G.gpio_handle < 0) {
        printf("[ERR] GPIO open failed\n");
        atomic_store(&G.running, 0);
        return;
    }
    lgGpioClaimOutput(G.gpio_handle, 0, R_STEP, 0);
    lgGpioClaimOutput(G.gpio_handle, 0, R_DIR,  0);
    lgGpioClaimOutput(G.gpio_handle, 0, P_STEP, 0);
    lgGpioClaimOutput(G.gpio_handle, 0, P_DIR,  0);

    printf("[MOTOR] Control loop started\n");

    while (atomic_load(&G.running)) {
        if (atomic_load(&G.emergency)) break;
        double now = time_now();

        if (atomic_load(&G.roll.pos) != atomic_load(&G.roll.target))
            motor_step_toward(&G.roll, R_STEP, R_DIR, now);
        else
            motor_correct(&G.roll, R_STEP, R_DIR, POSITION_TOLERANCE);

        if (atomic_load(&G.pitch.pos) != atomic_load(&G.pitch.target))
            motor_step_toward(&G.pitch, P_STEP, P_DIR, now);
        else
            motor_correct(&G.pitch, P_STEP, P_DIR, POSITION_TOLERANCE);
    }

    lgGpiochipClose(G.gpio_handle);
    printf("[MOTOR] Control loop stopped\n");
}
