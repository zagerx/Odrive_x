
#ifndef __OPEN_LOOP_CONTROLLER_LIB_H
#define __OPEN_LOOP_CONTROLLER_LIB_H

#include "MyProject.h"


/****************************************************************************/
typedef struct 
{
	float max_current_ramp_; // [A/s]
	float max_voltage_ramp_; // [V/s]
	float max_phase_vel_ramp_; // [rad/s^2]
	// Inputs
	float target_vel_;
	float target_current_;
	float target_voltage_;
	// State/Outputs
	//uint32_t timestamp_;
	float2D Idq_setpoint_;
	float2D Vdq_setpoint_;
	float phase_;
	float phase_vel_;
	float total_distance_;
} OPENLOOP_struct;

extern  OPENLOOP_struct openloop_controller_;
/****************************************************************************/
void openloop_controller_update(void);
/****************************************************************************/

#endif

