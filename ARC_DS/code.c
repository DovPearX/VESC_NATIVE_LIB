#include "vesc_c_if.h"

#include "biquad.h"
#include "datatypes.h"
#include "utils.h"
#include "motor_data.h"
#include "proportional_gain.h"
#include "traction.h"
#include "kalman.h"

HEADER

////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    lib_thread thread;
    math_config config;

    MotorData motor;

    float motor_timeout_s;

    // Runtime values grouped for easy access in ancillary functions
	RuntimeData rt; // pitch_angle proportional pid_value setpoint current_time roll_angle  last_accel_z  accel[3]

    // Rumtime state values
	State state;

    // Kalman Filter
	KalmanFilter pitch_kalman; 
	float pitch_smooth_kalman;
	float diff_time, last_time;

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

    //Pitch Kalman Configure
	configure_kalman(&d->config, &d->pitch_kalman);

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

    // Traction Control
	reset_traction(&d->traction, &d->state);

    //Low pass pitch filter
	d->prop_smooth = 0;
	d->abs_prop_smooth = 0;
	d->pitch_smooth = d->rt.pitch_angle;
	biquad_reset(&d->pitch_biquad);

    //Kalman filter
	reset_kalman(&d->pitch_kalman);
	d->pitch_smooth_kalman = d->rt.pitch_angle;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static void set_current(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_current(current);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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

// static void imu_ref_callback(float *acc, float *gyro, float *mag, float dt) {
// 	UNUSED(mag);
// 	data *d = (data*)ARG;
// 	VESC_IF->ahrs_update_mahony_imu(gyro, acc, dt, &d->m_att_ref);
// }

////////////////////////////////////////////////////////////////////////////////////////////////////

static void thd(void *arg) 
{
    data *d = (data*)arg;
    
    configure(d);

    while (!VESC_IF->should_terminate()) {

        // Update times
		d->rt.current_time = VESC_IF->system_time();
		if (d->last_time == 0) {
			d->last_time = d->rt.current_time;
		}
		d->diff_time = d->rt.current_time - d->last_time;
		d->last_time = d->rt.current_time;

        VESC_IF->sleep_ms(1000);
    }
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
