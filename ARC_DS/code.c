#include "vesc_c_if.h"

#include "biquad.h"
#include "datatypes.h"
#include "utils.h"
#include "motor_data.h"
#include "proportional_gain.h"
#include "traction.h"

HEADER

////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    lib_thread thread;
    math_config config;

    MotorData motor;

    // Runtime values grouped for easy access in ancillary functions
	RuntimeData rt; // pitch_angle proportional pid_value setpoint current_time roll_angle  last_accel_z  accel[3]

    // Rumtime state values
	State state;

    //Config values
    float mc_current_max, mc_current_min;

    // Low Pass Filter
	float pitch_smooth; 
	Biquad pitch_biquad;

	// Throttle/Brake Scaling
	float prop_smooth, abs_prop_smooth;
	float roll_pid_mod;
	KpArray accel_kp;
	KpArray brake_kp;
	KpArray roll_accel_kp;
	KpArray roll_brake_kp;
	KpArray yaw_accel_kp;
	KpArray yaw_brake_kp;

    //Traction Control
	TractionData traction;
	TractionDebug traction_dbg;

    //Runtime values
    float gyro[3];

} data;

////////////////////////////////////////////////////////////////////////////////////////////////////

static void thd(void *arg) 
{
    data *d = (data*)arg;

    while (!VESC_IF->should_terminate()) {

        VESC_IF->sleep_ms(1000);
    }
}

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

    //Pitch Biquad Configure
	biquad_configure(&d->pitch_biquad, BQ_LOWPASS, d->config.pitch_filter / d->config.hertz);

    //Motor Data Configure
	motor_data_configure(&d->motor, 3.0 / d->config.hertz);

    //initialize current and pitch arrays for acceleration
	angle_kp_reset(&d->accel_kp);
	pitch_kp_configure(&d->config, &d->accel_kp, 1);
	
	//initialize current and pitch arrays for braking
	if (d->config.brake_curve) {
		angle_kp_reset(&d->brake_kp);
		pitch_kp_configure(&d->config, &d->brake_kp, 2);
	}
	
	//Check for roll inputs
	roll_kp_configure(&d->config, &d->roll_accel_kp, 1);
	roll_kp_configure(&d->config, &d->roll_brake_kp, 2);

	//Check for yaw inputs
	yaw_kp_configure(&d->config, &d->yaw_accel_kp, 1);
	yaw_kp_configure(&d->config, &d->yaw_brake_kp, 2);

	//Traction Configure
	configure_traction(&d->traction, &d->config, &d->traction_dbg);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void reset_vars(data *d) 
{
    motor_data_reset(&d->motor);

    //Low pass pitch filter
	d->prop_smooth = 0;
	d->abs_prop_smooth = 0;
	d->pitch_smooth = d->rt.pitch_angle;
	biquad_reset(&d->pitch_biquad);

    // Traction Control
	reset_traction(&d->traction, &d->state);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Called when code is stopped
static void stop(void *arg) 
{
    data *d = (data*)arg;
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

    d->thread = VESC_IF->spawn(thd, 1024, "LibThd", d);
    if (!d->thread) {
        VESC_IF->printf("Failed to spawn LibThd thread.\n");
        VESC_IF->free(d);
        return false;
    }

    info->stop_fun = stop;
    info->arg = d;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
