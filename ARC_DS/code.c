#include "vesc_c_if.h"

#include "biquad.h"
#include "utils.h"
#include "motor_data.h"

HEADER

////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct {
    lib_thread thread;

    MotorData motor;

    //Config values
    float mc_current_max, mc_current_min;

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

    //Motor Data Configure
	motor_data_configure(&d->motor, 3.0 / d->tnt_conf.hertz);

}

////////////////////////////////////////////////////////////////////////////////////////////////////

// static void reset_vars(data *d) 
// {
//     d->gyro[0] = 0;
//     d->gyro[1] = 0;
//     d->gyro[2] = 0;
// }

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
