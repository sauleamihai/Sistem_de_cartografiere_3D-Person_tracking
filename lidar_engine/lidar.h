#ifndef LIDAR_H
#define LIDAR_H
#include <stdint.h>

typedef enum { AXIS_PITCH = 0, AXIS_ROLL = 1 } ScanAxis;

int  lidar_open_uart(void);
void color_from_depth(float z, uint8_t* r, uint8_t* g, uint8_t* b);

// Capture LiDAR for SCAN_CAPTURE_TIME at a fixed platform angle, parse the
// M1C1 packet stream, transform each valid point into world coordinates for
// the given axis, and stream points as UDP "PTS:" packets to the client.
// This single function replaces the duplicated pitch/roll capture blocks.
void lidar_capture_angle(int uart, int sock, ScanAxis axis, float actual_angle);

#endif
