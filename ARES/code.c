#include "vesc_c_if.h"

#include "datatypes.h"

#include <math.h>
#include <string.h>

HEADER

typedef struct {
    lib_thread thread;

    RuntimeData rt;
    State state;

} data;

////////////////////////////////////////////////////////////////////////////////////////////////////

float calculateResultantAcceleration(float accel_x, float accel_y) {
    return sqrt(accel_x * accel_x + accel_y * accel_y);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void manageVehicleControl(data *d) {
    const float MAX_ERPM_BASE = 10000;
    const float SPEED_FACTOR = 0.1;
    const float ACCEL_FACTOR = 50;
    const float MAX_GYRO_Y = 100.0;
    const float MAX_ACCEL = 3.0;
    const float BRAKE_FORCE = 0.8;
    const float DUTY_CYCLE_REDUCTION_FACTOR = 0.9;

    int rpm = VESC_IF->mc_get_rpm();

    float max_erpm_dynamic = MAX_ERPM_BASE + d->motor.acceleration * ACCEL_FACTOR + d->motor.current_avg * SPEED_FACTOR;

    if (d->state.wheelslip > 0) {
        max_erpm_dynamic *= 0.8;
    }

    float resultant_acceleration = calculateResultantAcceleration(d->accelxyz[0], d->accelxyz[1]);

    bool isSlipping = (abs(d->gyroxyz[1]) > MAX_GYRO_Y || resultant_acceleration > MAX_ACCEL);
    
    if (isSlipping) {
        set_current(d, BRAKE_FORCE);
        float new_duty_cycle = d->motor.duty_cycle * DUTY_CYCLE_REDUCTION_FACTOR;
        set_dutycycle(d, new_duty_cycle);
    } else {
        set_current(d, 0);
        
        if (d->motor.abs_erpm > max_erpm_dynamic) {
            float new_duty_cycle = d->motor.duty_cycle * DUTY_CYCLE_REDUCTION_FACTOR;
            set_dutycycle(d, new_duty_cycle);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void thd(void *arg) 
{
    data *d = (data*)arg;
    

    while (!VESC_IF->should_terminate()) {

        // Update times
		d->rt.current_time = VESC_IF->system_time();
		if (d->rt.last_time == 0) {
			d->rt.last_time = d->rt.current_time;
		}
		d->rt.diff_time = d->rt.current_time - d->rt.last_time;
		d->rt.last_time = d->rt.current_time;
		
		VESC_IF->imu_get_gyro(d->rt.gyroxyz);
		VESC_IF->imu_get_accel(d->rt.accelxyz);

        manageVehicleControl(d);

        VESC_IF->sleep_ms(10);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static float app_get_debug(int index) {
    data *d = (data *) ARG;

    switch (index) {
    case (1):
        return d->current_time;
    case (2):
        return d->gyroxyz[0];
    case (3):
        return d->gyroxyz[1];
    case (4):
        return d->gyroxyz[2];
    case (5):
        return d->accelxyz[0];
    case (6):
        return d->accelxyz[1];
    case (7):
        return d->accelxyz[2];
    case (8):
        return d->motor.duty_cycle;
    case (9):
        return d->motor.current;
    case (10):
        return d->motor.braking;
    case (11):
        return d->motor.abs_erpm; 
    case (12):
        return d->motor.acceleration;
    case (13):
        return d->motor.current_avg;
    case (14):
        return d->state.wheelslip;

    default:
        return 0;
    }
}

// Register get_debug as a lisp extension
static lbm_value ext_arc_debug(lbm_value *args, lbm_uint argn) {
	if (argn != 1 || !VESC_IF->lbm_is_number(args[0])) {
		return VESC_IF->lbm_enc_sym_eerror;
	}

	return VESC_IF->lbm_enc_float(app_get_debug(VESC_IF->lbm_dec_as_i32(args[0])));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Called when code is stopped
static void stop(void *arg) 
{
    data *d = (data*)arg;
	VESC_IF->imu_set_read_callback(NULL);
	VESC_IF->set_app_data_handler(NULL);
	VESC_IF->conf_custom_clear_configs();
    VESC_IF->request_terminate(d->thread);
    VESC_IF->printf("Terminated\n");
    VESC_IF->free(d);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

INIT_FUN(lib_info *info) 
{
    INIT_START

    data *d = VESC_IF->malloc(sizeof(data));
    if (!d) {
        VESC_IF->printf("Failed to allocate memory for data.\n");
        return false;
    }

    d->thread = VESC_IF->spawn(thd, 1024, "THD MAIN", d);
    if (!d->thread) {
        VESC_IF->printf("Failed to spawn THD MAIN thread.\n");
        VESC_IF->free(d);
        return false;
    }

	info->stop_fun = stop;
    info->arg = d;

	VESC_IF->lbm_add_extension("ext-arc-debug", ext_arc_debug);

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
