#ifndef MATHLUT_H
#define MATHLUT_H

// Precomputed sin/cos lookup table (0.25 deg resolution).
// The scan transform calls these per point; a LUT keeps it branch-light
// and avoids libm calls in the hot loop.

void  init_lut(void);
float fast_sin(float deg);
float fast_cos(float deg);

#endif
