#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// config.h - All compile-time configuration in one place.
// Pins, calibration constants, scan geometry, thermal limits, control gains.
// Nothing here allocates or executes; it is pure tunable configuration.
// ============================================================================

// ----- GPIO & hardware pins (lgpio chip 4 = RP1 on Raspberry Pi 5) -----
#define MOTOR_CHIP 4
#define R_STEP 23
#define R_DIR  24
#define P_STEP 25
#define P_DIR  16

#define FAN1_PWM 22
#define FAN1_IN1 17
#define FAN1_IN2 27
#define FAN2_PWM 26
#define FAN2_IN3 5
#define FAN2_IN4 6

// ----- I2C / UART / UDP -----
#define I2C_BUS_0 "/dev/i2c-0"
#define I2C_BUS_1 "/dev/i2c-1"
#define UART_PORT "/dev/ttyAMA0"

// NOTE: the original engine set the UART to 115200 in code while the README
// documented 230400. 115200 is what actually ran, so it is the default here.
// Change ONE place if your M1C1 is configured for 230400.
#define LIDAR_BAUD B115200

#define UDP_PORT_CMD  5005
#define UDP_PORT_DATA 5006

// ----- LiDAR mounting geometry (mm / deg) -----
#define LIDAR_PITCH_OFFSET 55.0f
#define LIDAR_ROLL_OFFSET  90.0f

// ----- Encoder calibration (AS5600) -----
#define ROLL_INVERTED      1
#define ROLL_SCALE_FACTOR  1.658029f
#define ROLL_OFFSET        (-0.3820f)

#define PITCH_INVERTED     1
#define PITCH_SCALE_FACTOR 1.042918f
#define PITCH_OFFSET       (+1.1387f)

// ----- IMU (MPU6050) -----
#define MPU6050_ADDR     0x68
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_ALPHA        0.96f
#define IMU_ROLL_OFFSET  (-1.8952f)
#define IMU_PITCH_OFFSET (+0.9779f)

// ----- Encoder (AS5600) -----
#define AS5600_ADDR        0x36
#define AS5600_ANGLE_REG   0x0C
#define ENCODER_POLL_FAST_MS 4     // 250 Hz while moving
#define ENCODER_POLL_SLOW_MS 50    // 20 Hz while idle
#define ENCODER_FILTER_SIZE  8

// ----- Trig lookup table -----
#define LUT_RES  0.25f
#define LUT_SIZE 1500

// ----- Stepper mechanics -----
#define STEPS_PER_REV 200
#define MICROSTEPS    32
#define STEPS_PER_DEG ((STEPS_PER_REV * MICROSTEPS) / 360.0)

#define ROLL_MAX_SPEED    150.0
#define ROLL_CRUISE_SPEED 100.0
#define ROLL_ACCELERATION  80.0
#define ROLL_MIN_SPEED     15.0

#define PITCH_MAX_SPEED   400.0
#define PITCH_CRUISE_SPEED 300.0
#define PITCH_ACCELERATION 200.0
#define PITCH_MIN_SPEED    20.0

// ----- Scan sweep settings -----
#define PITCH_MIN -45
#define PITCH_MAX  45
#define ROLL_MIN  -30
#define ROLL_MAX   30
#define SCAN_STEP            0.25f
#define SCAN_SETTLE_PITCH_MS 50
#define SCAN_SETTLE_ROLL_MS  80
#define SCAN_CAPTURE_TIME    0.25   // seconds of LiDAR capture per angle
#define SCAN_MIN_DIST 150
#define SCAN_MAX_DIST 7000
#define FLYING_PIXEL_THRESH 50

// ----- Thermal -----
#define HTU21D_ADDR 0x40
#define BMP280_ADDR 0x76
#define TEMP_MIN      30.0
#define TEMP_MAX      50.0
#define TEMP_CRITICAL 60.0

// ----- Closed-loop position correction -----
#define POSITION_TOLERANCE     2.0f
#define HOMING_TOLERANCE       0.5f
#define MAX_CORRECTION_STEPS   10
#define CORRECTION_COOLDOWN_MS 1000
#define CORRECTION_PULSE_US    800

// ----- RT thread layout (core, SCHED_FIFO priority) -----
#define CORE_NONRT   0   // temp, network, watchdog
#define CORE_SENSOR  1   // encoder, imu
#define CORE_SCAN    2   // scanner
#define CORE_MOTOR   3   // motor control (isolated)

#define PRIO_MOTOR   99
#define PRIO_SCAN    90
#define PRIO_ENCODER 80
#define PRIO_IMU     50
#define PRIO_NONRT    0

#endif // CONFIG_H
