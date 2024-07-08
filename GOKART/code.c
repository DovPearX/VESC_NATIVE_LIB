#include "vesc_c_if.h"

#include "datatypes.h"
#include "biquad.h"
#include "traction.h"
#include "utils.h"
#include "state.h"
#include "motor_data.h"

HEADER

typedef struct {
    lib_thread thread;
    config tnt_conf;

    MotorData motor;

    // Low Pass Filter
	float pitch_smooth; 
	Biquad pitch_biquad;

    //Debug
	float debug1, debug2, debug3, debug4, debug5, debug6;

    //Traction Control
	TractionData traction;
	TractionDebug traction_dbg;

    // Rumtime state values
	State state;

    // Runtime values grouped for easy access in ancillary functions
	RuntimeData rt; 		// pitch_angle proportional pid_value setpoint current_time roll_angle  last_accel_z  accel[3
} data;

static void thd(void *arg) {
    data *d = (data*)arg;

    while (!VESC_IF->should_terminate()) {

        // Pobierz wartości RPM
        d->rpm = VESC_IF->mc_get_rpm();
        VESC_IF->printf("RPM: %d\n", d->rpm);

        VESC_IF->sleep_ms(1000);
    }
}

// Called when code is stopped
static void stop(void *arg) {
    data *d = (data*)arg;
    VESC_IF->request_terminate(d->thread);
    VESC_IF->printf("Terminated\n");
    VESC_IF->free(d);
}

INIT_FUN(lib_info *info) {
    INIT_START

    data *d = VESC_IF->malloc(sizeof(data));
    if (!d) {
        VESC_IF->printf("Failed to allocate memory for data.\n");
        return false;
    }

    d->thread = VESC_IF->spawn(thd, 1024, "LibThd", d);
    if (!d->thread) {
        VESC_IF->printf("Failed to spawn LibThd thread.\n");
        VESC_IF->free(d);
        return false;
    }

    info->stop_fun = stop;
    info->arg = d;

    VESC_IF->printf("Hello Example!\n");

    return true;
}
