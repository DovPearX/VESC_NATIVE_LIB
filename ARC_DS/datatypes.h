#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {

    uint16_t hertz;
    float pitch_filter;
    bool brake_curve;

} math_config; 

typedef struct { //Run time values used in various features
	float proportional;
	float pid_value;
	float pitch_angle;
	float roll_angle;
	float current_time;
	float setpoint;
	float last_accel_z;
	float accel[3];
} RuntimeData;

#endif // DATATYPES_H_
