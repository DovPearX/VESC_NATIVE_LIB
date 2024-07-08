#pragma once
#include "motor_data.h"
#include "state.h"
#include "datatypes.h"

typedef struct {
	float timeron;       	 	//Timer from the start of wheelslip
	float timeroff;      		//Timer from the end of high motor acceleration
	float accelstartval;		//Starting value to engage wheelslip
	bool highaccelon;		//Flag that indicates acceleration direction has changed
	float lasterpm;			//ERPM before wheelslip
	float erpm;			//ERPM once wheelslip engaged
	bool reverse_wheelslip; 	//Wheelslip in the braking position
	float start_accel;		//acceleration that triggers wheelslip
	float slowed_accel;		//Trigger that shows traction control is working
} TractionData;

typedef struct {
	float debug1;
	float debug2;
	float debug3;
	float debug4;
	float debug5;
	float debug6;
	float debug7;
	float debug8;
	float debug9;
	float aggregate_timer;
	float freq_factor;
} TractionDebug;

void check_traction(MotorData *m, TractionData *traction, State *state, RuntimeData *rt, config *config, TractionDebug *traction_dbg);
void reset_traction(TractionData *traction, State *state);
void deactivate_traction(TractionData *traction, State *state, RuntimeData *rt, TractionDebug *traction_dbg);
void configure_traction(TractionData *traction, config *config, TractionDebug *traction_dbg);
