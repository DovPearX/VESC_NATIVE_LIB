#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	float version;

	float current_time, diff_time, last_time;

	float gyroxyz[3];
	float accelxyz[3];

} RuntimeData; 

typedef enum {
    STATE_DISABLED = 0,
    STATE_STARTUP = 1,
    STATE_READY = 2,
    STATE_RUNNING = 3
} RunState;

typedef struct { 
	RunState state;
} State;

#endif // DATATYPES_H_
