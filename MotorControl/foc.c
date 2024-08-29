


#include "MyProject.h"



/*****************************************************************************/
uint32_t  motor_error;
float  Ialpha_beta[2];

float2D  Idq_setpoint_;
float2D  Vdq_setpoint_;
float  phase_;
float  phase_vel_;
float2D  *Idq_setpoint_src_;
float2D  *Vdq_setpoint_src_;
float  *phase_src_;
float  *phase_vel_src_;

float  Id_measured,Iq_measured;   //[A]
float  I_measured_report_filter_k = 1.0f;
float  pi_gains_[2];
float  v_current_control_integral_d = 0.0f; // [V]
float  v_current_control_integral_q = 0.0f; // [V]
float  final_v_alpha = 0.0f; // [V]
float  final_v_beta = 0.0f;  // [V]
float  Ibus;
/*****************************************************************************/
bool enqueue_modulation_timings(float mod_alpha, float mod_beta)
{
	float tA, tB, tC;
	if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
	{
		set_error(ERROR_MODULATION_MAGNITUDE);
		return false;
	}
	TIM1->CCR1 = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR2 = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR3 = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
	return true;
}
/*****************************************************************************/
bool enqueue_voltage_timings(float v_alpha, float v_beta)
{
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = vfactor * v_alpha;
	float mod_beta = vfactor * v_beta;
	if (!enqueue_modulation_timings(mod_alpha, mod_beta))
		return false;
	return true;
}
/****************************************************************************/
// We should probably make FOC Current call FOC Voltage to avoid duplication.
//当前代码移植0.5.1
bool FOC_voltage(float v_d, float v_q, float pwm_phase)  //pwm_phase是park逆变换时的θ
{
	float c = our_arm_cos_f32(pwm_phase);
	float s = our_arm_sin_f32(pwm_phase);
	float v_alpha = c*v_d - s*v_q;
	float v_beta = c*v_q + s*v_d;
	return enqueue_voltage_timings(v_alpha, v_beta);
}
/*****************************************************************************/
void on_measurement(uint32_t input_timestamp, Iph_ABC_t *current)
{
	// Clark transform
	Ialpha_beta[0] = current->phA;
	Ialpha_beta[1] = one_by_sqrt3 * (current->phB - current->phC);
	
	if(meas_resis==1)Resistance_on_measurement();
	else if(meas_induc==1)Inductance_on_measurement(input_timestamp);
}
/*****************************************************************************/
void foc_reset(void)
{
	v_current_control_integral_d = 0.0f;  //积分清零，否则闭环时电机误动作
	v_current_control_integral_q = 0.0f;
	Ialpha_beta[0] = 0;
	Ialpha_beta[1] = 0;
}
/*****************************************************************************/
//0.5.6版本，clark变换与之后的变换分开处理
bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase)   //I_phase是park变换时的θ，pwm_phase是park逆变换时的θ
{
	// Park transform
	float c_I = our_arm_cos_f32(I_phase);
	float s_I = our_arm_sin_f32(I_phase);
	float Id = c_I * Ialpha_beta[0] + s_I * Ialpha_beta[1];
	float Iq = c_I * Ialpha_beta[1] - s_I * Ialpha_beta[0];
	Iq_measured += I_measured_report_filter_k * (Iq - Iq_measured);
	Id_measured += I_measured_report_filter_k * (Id - Id_measured);
	
	motor_debug.id_real = Id;
	motor_debug.iq_real = Iq;
	// Current error
	float Ierr_d = Id_des - Id;
	float Ierr_q = Iq_des - Iq;
	
	// TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
	// Apply PI control
	float Vd = v_current_control_integral_d + Ierr_d * pi_gains_[0];
	float Vq = v_current_control_integral_q + Ierr_q * pi_gains_[0];
	
	float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
	float V_to_mod = 1.0f / mod_to_V;
	float mod_d = V_to_mod * Vd;
	float mod_q = V_to_mod * Vq;
	
	// Vector modulation saturation, lock integrator if saturated
	// TODO make maximum modulation configurable
	float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
	if (mod_scalefactor < 1.0f)
	{
		mod_d *= mod_scalefactor;
		mod_q *= mod_scalefactor;
		// TODO make decayfactor configurable
		v_current_control_integral_d *= 0.99f;
		v_current_control_integral_q *= 0.99f;
	}
	else
	{
		v_current_control_integral_d += Ierr_d * (pi_gains_[1] * current_meas_period);
		v_current_control_integral_q += Ierr_q * (pi_gains_[1] * current_meas_period);
	}
	
	// Compute estimated bus current
	Ibus = mod_d * Id + mod_q * Iq;
	//power_ = vbus_voltage * Ibus;
	
	// Inverse park transform
	float c_p = our_arm_cos_f32(pwm_phase);
	float s_p = our_arm_sin_f32(pwm_phase);
	float mod_alpha = c_p * mod_d - s_p * mod_q;
	float mod_beta = c_p * mod_q + s_p * mod_d;
	
	// Report final applied voltage in stationary frame (for sensorles estimator)
	final_v_alpha= mod_to_V * mod_alpha;
	final_v_beta = mod_to_V * mod_beta;
	
	// Apply SVM
	if (!enqueue_modulation_timings(mod_alpha, mod_beta))
		return false; // error set inside enqueue_modulation_timings
	
	return true;
}
/*****************************************************************************/
void foc_update(void)
{
	if((Idq_setpoint_src_==NULL)||(Vdq_setpoint_src_==NULL))return;  //刚上电时还没有赋值指针，发送指令“A”才赋值
	
	Idq_setpoint_.d = Idq_setpoint_src_->d;
	Idq_setpoint_.q = Idq_setpoint_src_->q;
	Vdq_setpoint_.d = Vdq_setpoint_src_->d;
	Vdq_setpoint_.q = Vdq_setpoint_src_->q;
	phase_ = *phase_src_;
	phase_vel_ = *phase_vel_src_;
}
/*****************************************************************************/
//0.5.6使用了包含函数指针的类，切换不同工作状态，非常不直观。所以移植时更改为标志位判断
void pwm_update_cb(void)
{
	if(meas_resis==1)Resistance_get_alpha_beta_output();      //测量电阻时
	else if(meas_induc==1)Inductance_get_alpha_beta_output(); //测量电感时
	else   //正常工作时
	{
		float pwm_phase = phase_ + 1.5f * current_meas_period * phase_vel_;  //0.5.6是在反park变换时计算，效果一样
		
		if(motor_config.motor_type == MOTOR_TYPE_GIMBAL)
			FOC_voltage(Vdq_setpoint_.d, Vdq_setpoint_.q, pwm_phase);   //gimbal电机用电压模式
		else
			FOC_current(Idq_setpoint_.d, Idq_setpoint_.q, phase_, pwm_phase);  //highcurrent电机用电流模式
	}
}
/*****************************************************************************/



