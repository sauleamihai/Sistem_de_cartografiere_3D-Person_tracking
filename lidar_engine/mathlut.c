#include "mathlut.h"
#include "config.h"
#include <math.h>

static float sin_lut[LUT_SIZE];
static float cos_lut[LUT_SIZE];

void init_lut(void) {
    for (int i = 0; i < LUT_SIZE; i++) {
        double rad = (i * LUT_RES) * (M_PI / 180.0);
        sin_lut[i] = sinf(rad);
        cos_lut[i] = cosf(rad);
    }
}

float fast_sin(float deg) {
    while (deg < 0)        deg += 360.0f;
    while (deg >= 360.0f)  deg -= 360.0f;
    int idx = (int)(deg / LUT_RES);
    return sin_lut[idx < LUT_SIZE ? idx : LUT_SIZE - 1];
}

float fast_cos(float deg) {
    while (deg < 0)        deg += 360.0f;
    while (deg >= 360.0f)  deg -= 360.0f;
    int idx = (int)(deg / LUT_RES);
    return cos_lut[idx < LUT_SIZE ? idx : LUT_SIZE - 1];
}
