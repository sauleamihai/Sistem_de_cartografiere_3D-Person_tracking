#ifndef MOTOR_H
#define MOTOR_H
#include "types.h"

void motor_init(Motor* m, Encoder* enc, double max_spd, double cruise,
                double accel, double min_spd);
void motor_set_target(Motor* m, float angle);

// Runs on the main thread, pinned to the isolated motor core. Owns the GPIO
// handle. Drives both axes with a trapezoidal profile and applies a gentle
// encoder-based correction when an axis is idle.
void motor_control_loop(void);

#endif
