
#ifndef __CONTROLLER_LIB_H
#define __CONTROLLER_LIB_H

#include "MyProject.h"


/****************************************************************************/
typedef enum
{
	CONTROL_MODE_VOLTAGE_CONTROL = 0,
	CONTROL_MODE_TORQUE_CONTROL,
	CONTROL_MODE_VELOCITY_CONTROL,
	CONTROL_MODE_POSITION_CONTROL,
} ControlMode;

typedef enum
{
	INPUT_MODE_INACTIVE,
	INPUT_MODE_PASSTHROUGH,
	INPUT_MODE_VEL_RAMP,
	INPUT_MODE_TORQUE_RAMP,
	INPUT_MODE_POS_FILTER,
	INPUT_MODE_TRAP_TRAJ,
	INPUT_MODE_TUNING,
}InputMode;
/****************************************************************************/
typedef struct
{
	float frequency;
	float pos_amplitude;
	float vel_amplitude;
	float torque_amplitude;
} AUTOTURNING_t;

typedef struct
{
	ControlMode control_mode;         //see: ControlMode_t
	InputMode input_mode;             //see: InputMode_t
	float pos_gain;                   // [(turn/s) / turn]
	float vel_gain;                   // [Nm/(turn/s)]
	// float vel_gain,                // [Nm/(rad/s)] <sensorless example>
	float vel_integrator_gain;        // [Nm/(turn/s * s)]
	float vel_limit;                  // [turn/s] Infinity to disable.
	float vel_limit_tolerance;        // ratio to vel_lim. Infinity to disable.
	float vel_integrator_limit;       // Vel. integrator clamping value. Infinity to disable.
	float vel_ramp_rate;              // [(turn/s) / s]
	float torque_ramp_rate;           // Nm / sec
	bool circular_setpoints;
	float circular_setpoint_range;    // Circular range when circular_setpoints is true. [turn]
	//uint32_t steps_per_circular_range;
	float inertia;                    // [Nm/(turn/s^2)]
	float input_filter_bandwidth;     // [1/s]
	float homing_speed;               // [turn/s]
	
	float gain_scheduling_width;
	bool enable_gain_scheduling;
	bool enable_vel_limit;
	bool enable_overspeed_error;
	bool enable_torque_mode_vel_limit;  // 力矩模式下，如果电机转速超过vel_limit ，电机输出的力矩将会减小
	//uint8_t axis_to_mirror;
	//float mirror_ratio;
	//float torque_mirror_ratio;
	//uint8_t load_encoder_axis;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
	float mechanical_power_bandwidth; // [rad/s] filter cutoff for mechanical power for spinout detction
	float electrical_power_bandwidth; // [rad/s] filter cutoff for electrical power for spinout detection
	float spinout_electrical_power_threshold; // [W] electrical power threshold for spinout detection
	float spinout_mechanical_power_threshold; // [W] mechanical power threshold for spinout detection
} CONTROLLER_Config_t;

extern  AUTOTURNING_t  autotuning_;
extern  CONTROLLER_Config_t  ctrl_config;

extern  float *controller_pos_estimate_linear_src_;
extern  float *controller_pos_estimate_circular_src_;
extern  float *controller_vel_estimate_src_;
extern  float *controller_pos_wrap_src_; 
extern  float vel_integrator_torque_;
extern  float vel_setpoint_;
extern  float input_vel_;
extern  float input_pos_;
extern  float input_torque_;
extern  float torque_output_;
extern  bool input_pos_updated_;
/****************************************************************************/
void controller_config_default(void);
void controller_para_init(void);
void controller_reset(void);
bool control_mode_updated(void);
bool controller_update(void);
/****************************************************************************/

#endif

