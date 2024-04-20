
#ifndef __AXIS_LIB_H
#define __AXIS_LIB_H

#include "MyProject.h"


/****************************************************************************/
typedef enum
{
	AXIS_STATE_IDLE = 0,
	AXIS_STATE_MOTOR_CALIBRATION,
	AXIS_STATE_ENCODER_INDEX_SEARCH,
	AXIS_STATE_ENCODER_DIR_FIND,
	AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
	AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
	AXIS_STATE_LOCKIN_SPIN,
	AXIS_STATE_CLOSED_LOOP_CONTROL,
	AXIS_STATE_UNDEFINED,
} AXIS_State_Type;

typedef struct 
{
	bool startup_motor_calibration;   //<! run motor calibration at startup, skip otherwise
	bool startup_encoder_index_search; //<! run encoder index search after startup, skip otherwise,this only has an effect if encoder.config.use_index is also true
	bool startup_encoder_offset_calibration; //<! run encoder offset calibration after startup, skip otherwise
	bool startup_closed_loop_control; //<! enable closed loop control after calibration/startup
	bool startup_homing; //<! enable homing after calibration/startup
	//bool enable_sensorless_mode;
} AXIS_CONFIG;

extern  AXIS_State_Type  current_state_;
extern  AXIS_CONFIG  axis_config;
/****************************************************************************/
void run_state_machine_loop(void);
/****************************************************************************/

#endif

