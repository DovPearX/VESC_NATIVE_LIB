#include "vesc_c_if.h"

#include "datatypes.h"

#include <math.h>
#include <string.h>

HEADER

typedef struct {
    lib_thread thread;

    RuntimeData rt;

    int test1;
    int test2;
    int test3;

} data;

////////////////////////////////////////////////////////////////////////////////////////////////////

// Stałe i parametry do dostosowania
#define FRICTION_COEFFICIENT 0.1  // Współczynnik tarcia
#define MEMORY_FACTOR 2        // Współczynnik pamięci prędkości

void update_velocity_and_speed_in_x(data *d) {
    float g = 9.81; // Przybliżenie przyspieszenia ziemskiego w m/s²


    // Obliczenie zmiany prędkości na podstawie przyspieszenia w wymiarze x
    float delta_velocity_x = d->rt.accel[0] * g * d->rt.diff_time;

    // Aktualizacja prędkości w wymiarze x
    d->rt.velocity[0] += delta_velocity_x;

    // Zastosowanie tarcia, jeśli brak ruchu
    if (fabs(d->rt.accel[0]) < 0.001) { // Jeśli przyspieszenie jest bliskie zeru
        d->rt.velocity[0] *= (1.0 - FRICTION_COEFFICIENT); // Zastosowanie tarcia
    }

    // Obliczenie całkowitej prędkości (wektorowej) w jednej płaszczyźnie (tylko x)
    d->rt.speed = fabs(d->rt.velocity[0]); // Użyj fabs aby uzyskać wartość bezwzględną prędkości

    // Aktualizacja wartości prędkości z pamięcią
    d->rt.velocity[0] *= MEMORY_FACTOR;
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
		
		VESC_IF->imu_get_gyro(d->rt.gyro);
		VESC_IF->imu_get_accel(d->rt.accel);

        update_velocity_and_speed_in_x(d);

        VESC_IF->sleep_ms(10);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

static float app_get_debug(int index) {
    data *d = (data *) ARG;

    switch (index) {
    case (1):
        return d->rt.current_time;
    case (2):
        return d->rt.gyro[0];
    case (3):
        return d->rt.gyro[1];
    case (4):
        return d->rt.gyro[2];
    case (5):
        return d->rt.accel[0];
    case (6):
        return d->rt.accel[1];
    case (7):
        return d->rt.accel[2];
    case (8):
        return d->rt.speed;
    case (9):
        return d->rt.speed_kmh;
    case (10):
        return d->rt.velocity[0];
    case (11):
        return d->rt.velocity[2];
    

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
