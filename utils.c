#include "utils.h"

#include <math.h>

uint32_t rnd(uint32_t seed) {
    return seed * 1664525u + 1013904223u;
}

float lerp(float x1, float x2, float y1, float y2, float val){
    if (x2 - x1 == 0) {
        return y1;
    }
    float t= (val - x1)/(x2 - x1);
    return (y1 * (1 - t) + y2 * t);
}

void rate_limitf(float *value, float target, float step) {
    if (fabsf(target - *value) < step) {
        *value = target;
    } else if (target - *value > 0) {
        *value += step;
    } else {
        *value -= step;
    }
}

float clampf(float value, float min, float max) {
    const float m = value < min ? min : value;
    return m > max ? max : m;
}
