// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

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

float biquad_process(Biquad *biquad, float in) {
    float out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}

void biquad_configure(Biquad *biquad, BiquadType type, float frequency) {
    float K = tanf(M_PI * frequency);
    float Q = 0.707;  // maximum sharpness (0.5 = maximum smoothness)
    float norm = 1 / (1 + K / Q + K * K);
    if (type == BQ_LOWPASS) {
        biquad->a0 = K * K * norm;
        biquad->a1 = 2 * biquad->a0;
        biquad->a2 = biquad->a0;
    } else if (type == BQ_HIGHPASS) {
        biquad->a0 = 1 * norm;
        biquad->a1 = -2 * biquad->a0;
        biquad->a2 = biquad->a0;
    }
    biquad->b1 = 2 * (K * K - 1) * norm;
    biquad->b2 = (1 - K / Q + K * K) * norm;
}

void biquad_reset(Biquad *biquad) {
    biquad->z1 = 0;
    biquad->z2 = 0;
}

void apply_kalman(float in, float in_rate, float *out, float dt, KalmanFilter *k){
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
	// Step 1
	float rate = in_rate / 131 - k->bias; 
	*out += dt * rate;
	// Update estimation error covariance - Project the error covariance ahead
	// Step 2 
	k->P00 += dt * (dt * k->P11 - k->P01 - k->P10 + k->Q_angle);
	k->P01 -= dt * k->P11;
	k->P10 -= dt * k->P11;
	k->P11 += k->Q_bias * dt;
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Step 4
	float S = k->P00 + k->R_measure; // Estimate error
	// Calculate Kalman gain
	// Step 5
	float K0 = k->P00 / S; // Kalman gain - This is a 2x1 vector
	float K1 = k->P10 / S;
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	// Step 3
	float y = in - *out; // Angle difference
	// Step 6
	*out += K0 * y;
	k->bias += K1 * y;
	// Calculate estimation error covariance - Update the error covariance
	// Step 7
	float P00_temp = k->P00;
	float P01_temp = k->P01;
	k->P00 -= K0 * P00_temp;
	k->P01 -= K0 * P01_temp;
	k->P10 -= K1 * P00_temp;
	k->P11 -= K1 * P01_temp;
}

void configure_kalman(const MainData *config, KalmanFilter *k) {
	k->Q_angle = config->kalman_factor1/10000;
	k->Q_bias = config->kalman_factor2/10000;
	k->R_measure = config->kalman_factor3/100000;
}

void reset_kalman(KalmanFilter *k) {
	k->P00 = 0;
	k->P01 = 0;
	k->P10 = 0;
	k->P11 = 0;
	k->bias = 0;
}

