
#ifndef __MOTOR_LIB_H
#define __MOTOR_LIB_H

#include "MyProject.h"


/****************************************************************************/
#define  MOTOR_TYPE_HIGH_CURRENT   0
#define  MOTOR_TYPE_GIMBAL         1
/****************************************************************************/
typedef struct 
{
	bool pre_calibrated;           // can be set to true to indicate that all values here are valid
	int32_t pole_pairs;
	float calibration_current;     // [A]
	float resistance_calib_max_voltage; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
	float phase_inductance;        // to be set by measure_phase_inductance
	float phase_resistance;        // to be set by measure_phase_resistance
	float torque_constant;         // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
	int32_t motor_type;
	// Read out max_allowed_current to see max supported value for current_lim.
	float current_lim;            //[A]
	float current_lim_margin;     // Maximum violation of current_lim
//	float torque_lim;             //[Nm]. 
	// Value used to compute shunt amplifier gains
	//float requested_current_range; //  [A]1mΩ采样电阻对应60A，0.5mΩ采样电阻对应120A  loop222.5
	float current_control_bandwidth;  // [rad/s]
	float inverter_temp_limit_lower;
	float inverter_temp_limit_upper;
	
//	float acim_gain_min_flux = 10;   // [A]
//	float acim_autoflux_min_Id = 10; // [A]
//	bool acim_autoflux_enable = false;
//	float acim_autoflux_attack_gain = 10.0f;
//	float acim_autoflux_decay_gain = 1.0f;
	
	bool R_wL_FF_enable;   // Enable feedforwards for R*I and w*L*I terms
	bool bEMF_FF_enable;   // Enable feedforward for bEMF
	
	float I_bus_hard_min;
	float I_bus_hard_max;
	float I_leak_max;
	
	float dc_calib_tau;
} MOTOR_CONFIG;

extern  MOTOR_CONFIG  motor_config;
extern  Iph_ABC_t  current_meas_;
extern  bool  meas_resis;
extern  bool  meas_induc;

extern  bool  is_armed_;
extern  bool  is_calibrated_;

extern  float *motor_torque_setpoint_src_;
extern  float *motor_phase_vel_src_;
extern  float2D  motor_Vdq_setpoint_;
extern  float2D  motor_Idq_setpoint_;
/****************************************************************************/
void motor_para_init(void);
void motor_setup(void);
void arm(void);
void disarm(void);
void Resistance_on_measurement(void);
void Inductance_on_measurement(uint32_t input_timestamp);
void Resistance_get_alpha_beta_output(void);
void Inductance_get_alpha_beta_output(void);
bool run_calibration(void);

void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current);
void dc_calib_cb(Iph_ABC_t *current);
float motor_max_available_torque(void);
void motor_update(void);
void update_current_controller_gains(void);
/****************************************************************************/

#endif

