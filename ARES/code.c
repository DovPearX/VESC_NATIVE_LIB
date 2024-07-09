#include "vesc_c_if.h"

#include "datatypes.h"
#include "utils.h"
#include "motor_data.h"
#include "traction.h"

#include <math.h>
#include <string.h>

HEADER

typedef struct {
    lib_thread thread;

    MainData values;

    MotorData motor;

	State state;

	//Traction Control
	TractionData traction;
	TractionDebug traction_dbg;

    //Config values
    float mc_current_max, mc_current_min;

	// Runtime values
	float gyroxyz[3];
	float accelxyz[3];

	// Misc
	float motor_timeout_s;
	float current_time, diff_time, last_time;

} data;

////////////////////////////////////////////////////////////////////////////////////////////////////

static void configure(data *d) 
{
    
    if (VESC_IF->get_cfg_float(CFG_PARAM_IMU_mahony_kp) > 1) {
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_kp, 0.4);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_ki, 0);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_accel_confidence_decay, 0.1);
    }

    d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));

    //Motor Data Configure
	motor_data_configure(&d->motor, 3.0 / d->values.hertz);

	//Traction Configure
	configure_traction(&d->traction, &d->values, &d->traction_dbg);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void reset_vars(data *d) 
{
    motor_data_reset(&d->motor);

    // Traction Control
	reset_traction(&d->traction, &d->state);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void set_current(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_current(current);
}

static void set_dutycycle(data *d, float dutycycle){
	// Limit duty output to configured max output
	if (dutycycle >  VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty)) {
		dutycycle = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty);
	} else if(dutycycle < 0 && dutycycle < -VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty)) {
		dutycycle = -VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty);
	}
	
	VESC_IF->timeout_reset();
	VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
	VESC_IF->mc_set_duty(dutycycle); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void thd(void *arg) 
{
    data *d = (data*)arg;
    
    //configure(d);

    while (!VESC_IF->should_terminate()) {

        // Update times
		d->current_time = VESC_IF->system_time();
		if (d->last_time == 0) {
			d->last_time = d->current_time;
		}
		d->diff_time = d->current_time - d->last_time;
		d->last_time = d->current_time;
		
		VESC_IF->imu_get_gyro(d->gyroxyz);
		VESC_IF->imu_get_accel(d->accelxyz);

        VESC_IF->sleep_ms(100);
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
