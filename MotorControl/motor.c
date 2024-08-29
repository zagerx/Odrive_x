
#include "MyProject.h"


/****************************************************************************/
MOTOR_CONFIG  motor_config;

bool  is_armed_ = false;
uint8_t armed_state_ = 0;
bool  is_calibrated_ = false; // Set in apply_config()
Iph_ABC_t  current_meas_;
Iph_ABC_t  DC_calib_;
float dc_calib_running_since_ = 0.0f;

float effective_current_lim_ = 10.0f; // [A]
float max_allowed_current_ = 0.0f;    // [A] set in setup()
float max_dc_calib_ = 0.0f;           // [A] set in setup()

float *motor_torque_setpoint_src_; // Usually points to the Controller object's output
float *motor_phase_vel_src_;       // Usually points to the Encoder object's output
float2D  motor_Vdq_setpoint_;
float2D  motor_Idq_setpoint_;

bool  meas_resis = false;   //当前正在测量电阻
bool  meas_induc = false;   //当前正在测量电感
/****************************************************************************/
void arm(void);
void disarm(void);
/****************************************************************************/
/****************************************************************************/
//参数初始化，官方代码中，直接在定义的时候赋值
void motor_para_init(void)
{
	motor_config.pre_calibrated = false; // can be set to true to indicate that all values here are valid
	motor_config.pole_pairs = MOTOR_pole_pairs;  //电机极对数，参数设置宏定义在MyProject.h文件中
	motor_config.calibration_current = MOTOR_calibration_current;    // [A]
	motor_config.resistance_calib_max_voltage = MOTOR_resistance_calib_max_voltage; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
	motor_config.phase_inductance = 0.0f;        // to be set by measure_phase_inductance
	motor_config.phase_resistance = 0.0f;        // to be set by measure_phase_resistance
	motor_config.torque_constant = 0.04f;        // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
	motor_config.motor_type = MOTOR_type;  
	// Read out max_allowed_current to see max supported value for current_lim.
	motor_config.current_lim = MOTOR_current_lim; //[A] 电机最大运行电流
	motor_config.current_lim_margin = 8.0f;      // Maximum violation of current_lim
//	motor_config.torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
	// Value used to compute shunt amplifier gains
	//float requested_current_range = 60.0f; //  [A]1mΩ采样电阻对应60A，0.5mΩ采样电阻对应120A
	motor_config.current_control_bandwidth = 1000.0f;  // [rad/s]
	motor_config.inverter_temp_limit_lower = 100;
	motor_config.inverter_temp_limit_upper = 120;
	
	motor_config.R_wL_FF_enable = false; // Enable feedforwards for R*I and w*L*I terms
	motor_config.bEMF_FF_enable = false; // Enable feedforward for bEMF
	
	motor_config.I_bus_hard_min = -INFINITY;
	motor_config.I_bus_hard_max = INFINITY;
	motor_config.I_leak_max = 0.1f;
	
	motor_config.dc_calib_tau = 0.2f;
}
/****************************************************************************/
void motor_setup(void)
{
	// Solve for exact gain, then snap down to have equal or larger range as requested
	// or largest possible range otherwise
	const float kMargin = 0.90f;
	const float max_output_swing = 1.35f; // [V] out of amplifier
	
	float max_unity_gain_current = kMargin * max_output_swing * (1/SHUNT_RESISTANCE); // [A]  1215
	//float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
	const float actual_gain=20.0f;  //固定放大倍数20倍
	
	// Values for current controller
	float phase_current_rev_gain_ = 1.0f / actual_gain;  //0.05
	// Clip all current control to actual usable range
	max_allowed_current_ = max_unity_gain_current * phase_current_rev_gain_;  // =1215*0.05=60.75 A

	max_dc_calib_ = 0.1f * max_allowed_current_;  //约等于 6A
}
/****************************************************************************/
float  cali_target_current_;
float  cali_max_voltage_;
float  actual_current_;
float  test_voltage_;
float  I_beta_; // [A] low pass filtered Ibeta response
float  test_mod_;
/****************************************************************************/
void Resistance_reset(void)
{
	test_voltage_ = 0;
	I_beta_ = 0;
}
/*************************************/
void Resistance_on_measurement(void)
{
	const float kI = 1.0f; // [(V/s)/A]
	const float kIBetaFilt = 80.0f;
	
	actual_current_ = Ialpha_beta[0];    //Ialpha_beta 是foc.c中clark变换后的值
	test_voltage_ += (kI * current_meas_period) * (cali_target_current_ - actual_current_);
	I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta[1] - I_beta_);
	if(fabsf(test_voltage_) > cali_max_voltage_)
	{
		disarm();
		set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
	}
	else if(vbus_voltage <= 6)   //限制电源电压不能低于6V
	{
		disarm();
		set_error(ERROR_UNKNOWN_VBUS_VOLTAGE);
	}
	else
	{
		float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
		test_mod_ = test_voltage_ * vfactor;
	}
}
/*************************************/
void Resistance_get_alpha_beta_output(void)
{
	float mod_alpha = test_mod_;
	float mod_beta = 0;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
uint32_t  start_timestamp_;
uint32_t  last_input_timestamp_;
float last_Ialpha_;
float deltaI_;
bool  attached_ = false;
/****************************************************************************/
void Inductance_reset(void)
{
	attached_ = 0;
	last_Ialpha_ = 0;
	deltaI_ = 0;
}
/*************************************/
void Inductance_on_measurement(uint32_t input_timestamp)
{
	if(attached_)
	{
		float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
		deltaI_ += -sign * (Ialpha_beta[0] - last_Ialpha_);
	}
	else
	{
		start_timestamp_ = input_timestamp;   //第一次进入这个函数时的时间
		attached_ = 1;
	}
	
	last_Ialpha_ = Ialpha_beta[0];
	last_input_timestamp_ = input_timestamp;  //每进入一次记录一次，最后一次记录的就是最后一次的时间
}
/*************************************/
void Inductance_get_alpha_beta_output(void)
{
	test_voltage_ *= -1.0f;
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = test_voltage_ * vfactor;
	float mod_beta = 0.0f;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
void update_current_controller_gains(void)
{
	// Calculate current control gains
	float p_gain = motor_config.current_control_bandwidth * motor_config.phase_inductance;
	float plant_pole = motor_config.phase_resistance / motor_config.phase_inductance;
	
	if(motor_config.phase_inductance == 0)plant_pole=0;  //针对云台电机，不测量电阻电感，防止I参数无穷大
	pi_gains_[0] = p_gain;
	pi_gains_[1] = plant_pole * p_gain;
}
/****************************************************************************/
bool measure_phase_resistance(float test_current, float max_voltage)
{
	uint32_t i;
	
	cali_target_current_ = test_current;
	cali_max_voltage_ = max_voltage;
	Resistance_reset();
	meas_resis = 1;
	
	arm();
	
	for(i = 0; i < 3000; ++i)
	{
		//if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_))
		if(!is_armed_)break;
		delay_us(1000);   //1ms
	}
	
	bool success = is_armed_;
	disarm();
	
	if(success)motor_config.phase_resistance = test_voltage_ / test_current;
	
	if(motor_config.phase_resistance > 2)    //MOTOR_TYPE_HIGH_CURRENT的相电阻要小于2Ω，否则就是gimbal
	{
		set_error(ERROR_NOT_HIGH_CURRENT_MOTOR);
		success = 0;
	}
	
	if((fabsf(I_beta_) / test_current) > 0.2f)
	{
		set_error(ERROR_UNBALANCED_PHASES);
		success = 0;
	}
	
	meas_resis = 0;
	return success;
}
/****************************************************************************/
bool measure_phase_inductance(float test_voltage)
{
	uint32_t i;
	
	test_voltage_ = test_voltage;
	Inductance_reset();
	meas_induc = 1;
	
	arm();
	
	for(i = 0; i < 1250; ++i)
	{
		//if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_))
		if(!is_armed_)break;
		delay_us(1000);
	}
	
	bool success = is_armed_;
	disarm();
	
	if(success)
	{
		float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ; // at 216MHz this overflows after 19 seconds
		motor_config.phase_inductance = fabsf(test_voltage_) / (deltaI_ / dt);
	}
	
	// TODO arbitrary values set for now
	if (!(motor_config.phase_inductance >= 2e-6f && motor_config.phase_inductance <= 4000e-6f))
	{
		set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE);
		success = 0;
	}
	
	meas_induc = 0;
	return success;
}
/****************************************************************************/
bool run_calibration(void)
{
	float R_calib_max_voltage = motor_config.resistance_calib_max_voltage;
	
	if(motor_config.motor_type == MOTOR_TYPE_HIGH_CURRENT)
	{
		if(!measure_phase_resistance(motor_config.calibration_current, R_calib_max_voltage))return 0;
		if(!measure_phase_inductance(R_calib_max_voltage))return 0;
	}
	else if(motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
		__nop();// no calibration needed
	}
	else
	{
		return 0;
	}
	
	update_current_controller_gains();
	
	is_calibrated_ = 1;
	return 1;
}
/****************************************************************************/
void arm(void)
{
	armed_state_ = 1;
	is_armed_ = 1;
	controller_reset();
	foc_reset();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);   //使能输出
}
/*************************************/
void disarm(void)
{
	is_armed_ = 0;
	armed_state_ = 0;
	TIM_CtrlPWMOutputs(TIM1, DISABLE);  //停止输出
}
/****************************************************************************/
bool dc_calib_valid;
/****************************************************************************/
void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	dc_calib_valid = (dc_calib_running_since_ >= motor_config.dc_calib_tau * 7.5f)    //默认7.5-需1.5秒
												&& (fabsf(DC_calib_.phA) < max_dc_calib_)
												&& (fabsf(DC_calib_.phB) < max_dc_calib_)
												&& (fabsf(DC_calib_.phC) < max_dc_calib_);
	if (armed_state_ == 1 || armed_state_ == 2)   //arm()后先空闲2次，大概是为了先让控制运算运行2次
	{
		current_meas_.phA = 0;
		current_meas_.phB = 0;
		current_meas_.phC = 0;
		armed_state_ += 1;
	}
	else if(dc_calib_valid)
	{
		current_meas_.phA = current->phA - DC_calib_.phA;
		current_meas_.phB = current->phB - DC_calib_.phB;
		current_meas_.phC = current->phC - DC_calib_.phC;
	}
	else
	{
		current_meas_.phA = 0;
		current_meas_.phB = 0;
		current_meas_.phC = 0;
	}
	
	motor_debug.ia_real = current_meas_.phA;
	motor_debug.ib_real = current_meas_.phB;
	motor_debug.ic_real = current_meas_.phC;
	
	on_measurement(timestamp, &current_meas_);
}
/****************************************************************************/
void dc_calib_cb(Iph_ABC_t *current)
{
	const float dc_calib_period = (float)(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;  //0.000125
	const float calib_filter_k = dc_calib_period / motor_config.dc_calib_tau;    //0.000625
	
	DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
	DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
	DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
	dc_calib_running_since_ += dc_calib_period;
}
/****************************************************************************/
//0.5.6官方代码中，此函数被do_check()函数调用，最终在TIM1更新中断中被调用，
//移植后的代码，放在motor_update()函数中，最终在TIM1更新中断中被调用，
//计算电流限制值
float motor_effective_current_lim(void)
{
	// Configured limit
	float current_lim = motor_config.current_lim;
	// Hardware limit
	if (motor_config.motor_type == MOTOR_TYPE_GIMBAL)
		current_lim = min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control，云台电机的电压不超过 12V/1.732=6.9，防止超调
	else
		current_lim = min(current_lim, max_allowed_current_);
	
	// Apply thermistor current limiters，
	//根据电机和MOS的温度限电流，此功能有很高实用价值，但没有移植，loop222,20230809
//	current_lim = min(current_lim, motor_thermistor_.get_current_limit(config_.current_lim));
//	current_lim = min(current_lim, fet_thermistor_.get_current_limit(config_.current_lim));
	effective_current_lim_ = current_lim;
	
	return effective_current_lim_;
}
/****************************************************************************/
//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
float motor_max_available_torque(void)
{
//	float max_torque = effective_current_lim_ * motor_config.torque_constant;
//	max_torque = clamp(max_torque, 0.0f, motor_config.torque_lim);
//	return max_torque;
	
	return effective_current_lim_ * motor_config.torque_constant;
}
/****************************************************************************/
/****************************************************************************/
void motor_update(void)
{
	// Load torque setpoint, convert to motor direction
	float *maybe_torque;
	maybe_torque = motor_torque_setpoint_src_;
	if (!has_value(maybe_torque))
	{
		return;
	}
	float torque = encoder_config.direction * *maybe_torque;
	
	// Load setpoints from previous iteration.
	float id = motor_Idq_setpoint_.d;
	float iq = motor_Idq_setpoint_.q;
	
	// Load effective current limit
	motor_effective_current_lim();    //计算电流限制值
	float ilim = effective_current_lim_;
	
	id = clamp(id, -ilim*0.99f, ilim*0.99f); // 1% space reserved for Iq to avoid numerical issues
	iq = torque / motor_config.torque_constant;
	
	// 2-norm clamping where Id takes priority
	float iq_lim_sqr = SQ(ilim) - SQ(id);
	float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
	iq = clamp(iq, -Iq_lim, Iq_lim);
	
	if (motor_config.motor_type != MOTOR_TYPE_GIMBAL)
	{
		motor_Idq_setpoint_.d = id;
		motor_Idq_setpoint_.q = iq;
	}
	
	float vd = 0.0f;
	float vq = 0.0f;
	float *phase_vel;
	phase_vel = motor_phase_vel_src_;
	
	if (motor_config.R_wL_FF_enable)  //电流环解藕前馈补偿
	{
		if (!has_value(phase_vel))
		{
			motor_error |= ERROR_UNKNOWN_PHASE_VEL;
			return;
		}
		vd -= *phase_vel * motor_config.phase_inductance * iq;
		vq += *phase_vel * motor_config.phase_inductance * id;
		vd += motor_config.phase_resistance * id;
		vq += motor_config.phase_resistance * iq;
	}
	
	if (motor_config.bEMF_FF_enable)  //反电动势补偿
	{
		if (!has_value(phase_vel))
		{
			motor_error |= ERROR_UNKNOWN_PHASE_VEL;
			return;
		}
		vq += *phase_vel * (2.0f/3.0f) * (motor_config.torque_constant / motor_config.pole_pairs);
	}
	
	if (motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
		// reinterpret current as voltage
		motor_Vdq_setpoint_.d = vd + id;
		motor_Vdq_setpoint_.q = vq + iq;
	}
	else
	{
		motor_Vdq_setpoint_.d = vd;
		motor_Vdq_setpoint_.q = vq;
	}
}
/****************************************************************************/



