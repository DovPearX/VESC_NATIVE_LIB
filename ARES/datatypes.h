#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float version;

	float current_time, diff_time, last_time;

	float gyro[3];
	float accel[3];
	float velocity[3];
	float speed;
	float speed_kmh;

} RuntimeData; 

#endif // DATATYPES_H_
