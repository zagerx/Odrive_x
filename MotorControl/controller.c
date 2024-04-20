
#include "MyProject.h"


/****************************************************************************/
AUTOTURNING_t  autotuning_;
CONTROLLER_Config_t  ctrl_config;

uint32_t  control_error;
//float last_error_time_ = 0.0f;
// Inputs
float *controller_pos_estimate_linear_src_ = NULL;
float *controller_pos_estimate_circular_src_ = NULL;
float *controller_vel_estimate_src_ = NULL;
float *controller_pos_wrap_src_ = NULL; 

float pos_setpoint_ = 0.0f; // [turns]
float vel_setpoint_ = 0.0f; // [turn/s]
float vel_integrator_torque_ = 0.0f;    // [Nm]
float torque_setpoint_ = 0.0f;  // [Nm]

float input_pos_ = 0.0f;     // [turns]
float input_vel_ = 0.0f;     // [turn/s]
float input_torque_ = 0.0f;  // [Nm]
float input_filter_kp_ = 0.0f;
float input_filter_ki_ = 0.0f;

float autotuning_phase_ = 0.0f;

bool input_pos_updated_ = false;

bool trajectory_done_ = true;

float mechanical_power_ = 0.0f; // [W]
float electrical_power_ = 0.0f; // [W]

// Outputs
float torque_output_ = 0.0f;
/****************************************************************************/
void Controller_update_filter_gains(void);  //函数声明
/****************************************************************************/
void controller_config_default(void)
{
	ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
	ctrl_config.input_mode = INPUT_MODE_PASSTHROUGH;  //see: InputMode_t
	ctrl_config.pos_gain = 20.0f;                  // [(turn/s) / turn]
	ctrl_config.vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
	ctrl_config.vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
	ctrl_config.vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
	ctrl_config.vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
	ctrl_config.vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
	ctrl_config.vel_ramp_rate = 1.0f;              // [(turn/s) / s]
	ctrl_config.torque_ramp_rate = 0.01f;          // Nm / sec
	ctrl_config.circular_setpoints = 0;
	ctrl_config.circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
	//ctrl_config.steps_per_circular_range = 1024;
	ctrl_config.inertia = 0.0f;                    // [Nm/(turn/s^2)]
	ctrl_config.input_filter_bandwidth = 2.0f;     // [1/s]
	ctrl_config.homing_speed = 0.25f;              // [turn/s]
	
	ctrl_config.gain_scheduling_width = 10.0f;
	ctrl_config.enable_gain_scheduling = false;
	ctrl_config.enable_vel_limit = true;
	ctrl_config.enable_overspeed_error = true;
	ctrl_config.enable_torque_mode_vel_limit = true;  // 力矩模式下，如果电机转速超过vel_limit ，电机输出的力矩将会减小
	//ctrl_config.axis_to_mirror = -1;
	//ctrl_config.mirror_ratio = 1.0f;
	//ctrl_config.torque_mirror_ratio = 0.0f;
	//ctrl_config.load_encoder_axis = -1;  // default depends on Axis number and is set in load_configuration(). Set to -1 to select sensorless estimator.
	ctrl_config.mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
	ctrl_config.electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
	ctrl_config.spinout_electrical_power_threshold = 10.0f;  // [W] electrical power threshold for spinout detection
	ctrl_config.spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection
	
	Controller_update_filter_gains();
}
/**************************************/
void controller_para_init(void)
{
	ctrl_config.control_mode = CONTROL_mode;         //控制模式
	ctrl_config.input_mode = INPUT_mode;             //输入模式
	ctrl_config.torque_ramp_rate = TORQUE_ramp_rate; //Nm / sec，力矩爬升率
	ctrl_config.vel_ramp_rate = VELOCITY_ramp_rate;  //速度的爬升率
	ctrl_config.vel_gain = VELOCITY_P;               //速度P参数
	ctrl_config.vel_integrator_gain = VELOCITY_I;    //速度I参数
	ctrl_config.vel_limit = VELOCITY_limit;          //最大转速限制，圈/秒
	ctrl_config.pos_gain = POSITION_P;               //位置P参数
}
/****************************************************************************/
//arm() 函数中调用一次
void controller_reset(void) 
{
	// pos_setpoint is initialized in start_closed_loop_control
	vel_setpoint_ = 0.0f;
	vel_integrator_torque_ = 0.0f;
	torque_setpoint_ = 0.0f; 
	mechanical_power_ = 0.0f;
	electrical_power_ = 0.0f;
}
/****************************************************************************/
void move_to_pos(float goal_point)
{
	planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_, trapTraj_config.vel_limit, trapTraj_config.accel_limit, trapTraj_config.decel_limit);
	trap_traj_.t_ = 0.0f;
	trajectory_done_ = false;
}
/****************************************************************************/
bool control_mode_updated(void)
{
	float *estimate;
	
	if (ctrl_config.control_mode >= CONTROL_MODE_POSITION_CONTROL)
	{
		estimate = (ctrl_config.circular_setpoints ? controller_pos_estimate_circular_src_ : controller_pos_estimate_linear_src_);
		pos_setpoint_ = *estimate;
		input_pos_ = *estimate;    //把当前位置做为目标位置，保证闭环后电机不动
	}
	return true;
}
/****************************************************************************/
//生成的两个变量用于 INPUT_MODE_POS_FILTER 模式
void Controller_update_filter_gains(void)
{
	float bandwidth = min(ctrl_config.input_filter_bandwidth, 0.25f * current_meas_hz);
	input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
	input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}
/****************************************************************************/
static float limitVel(float vel_limit, float vel_estimate, float vel_gain, float torque)
{
	float Tmax = (vel_limit - vel_estimate) * vel_gain;
	float Tmin = (-vel_limit - vel_estimate) * vel_gain;
	return clamp(torque, Tmin, Tmax);
}
/****************************************************************************/
bool controller_update(void)
{
	float *pos_estimate_linear;
	float *pos_estimate_circular;
	float *pos_wrap;
	float *vel_estimate;
		
	pos_estimate_linear = controller_pos_estimate_linear_src_;
	pos_estimate_circular = controller_pos_estimate_circular_src_;
	pos_wrap = controller_pos_wrap_src_;
	vel_estimate = controller_vel_estimate_src_;
	
	if (!has_value(vel_estimate))return 0;
	
	// TODO also enable circular deltas for 2nd order filter, etc.
	if (ctrl_config.circular_setpoints)
	{
		if (!has_value(pos_wrap))
		{
			set_error(ERROR_INVALID_CIRCULAR_RANGE);
			return 0;
		}
		input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
	}
	
	switch (ctrl_config.input_mode)
	{
		case INPUT_MODE_INACTIVE: {  //不起作用模式，不赋值给设定值
			// do nothing
			} break;
		
		case INPUT_MODE_PASSTHROUGH: {  //直通模式，输入指令直接赋值给设定值
			pos_setpoint_ = input_pos_;
			vel_setpoint_ = input_vel_;
			torque_setpoint_ = input_torque_; 
		} break;
		
		case INPUT_MODE_VEL_RAMP: {  //速度爬升模式
			float max_step_size = fabsf(current_meas_period * ctrl_config.vel_ramp_rate);
			float full_step = input_vel_ - vel_setpoint_;
			float step = clamp(full_step, -max_step_size, max_step_size);
			vel_setpoint_ += step;
			torque_setpoint_ = (step / current_meas_period) * ctrl_config.inertia;
		} break;
		
		case INPUT_MODE_TORQUE_RAMP: {  //力矩爬升模式
			float max_step_size = fabsf(current_meas_period * ctrl_config.torque_ramp_rate);
			float full_step = input_torque_ - torque_setpoint_;
			float step = clamp(full_step, -max_step_size, max_step_size);
			torque_setpoint_ += step;
		} break;
		
		case INPUT_MODE_POS_FILTER: {  //位置平滑模式
			// 2nd order pos tracking filter
			float delta_pos = input_pos_ - pos_setpoint_; // Pos error
			if (ctrl_config.circular_setpoints)
			{
				delta_pos = wrap_pm(delta_pos, *pos_wrap);
			}
			float delta_vel = input_vel_ - vel_setpoint_;   // Vel error
			float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
			torque_setpoint_ = accel * ctrl_config.inertia; // Accel
			vel_setpoint_ += current_meas_period * accel;   // delta vel
			pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
		} break;
		
		case INPUT_MODE_TRAP_TRAJ: {  //梯形轨迹
			if(input_pos_updated_)
			{
				move_to_pos(input_pos_);
				input_pos_updated_ = false;
			}
			// Avoid updating uninitialized trajectory
			if (trajectory_done_)break;
			
			if (trap_traj_.t_ > trap_traj_.Tf_)
			{
				// Drop into position control mode when done to avoid problems on loop counter delta overflow
				ctrl_config.control_mode = CONTROL_MODE_POSITION_CONTROL;
				pos_setpoint_ = trap_traj_.Xf_;
				vel_setpoint_ = 0.0f;
				torque_setpoint_ = 0.0f;
				trajectory_done_ = true;
			}
			else
			{
				Step_t  traj_step = trap_traj_eval(trap_traj_.t_);
				pos_setpoint_ = traj_step.Y;
				vel_setpoint_ = traj_step.Yd;
				torque_setpoint_ = traj_step.Ydd * ctrl_config.inertia;
				trap_traj_.t_ += current_meas_period;
			}
		} break;
		
		case INPUT_MODE_TUNING: {  //单圈循环模式，只在一圈内转动
			autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
			float c = our_arm_cos_f32(autotuning_phase_);
			float s = our_arm_sin_f32(autotuning_phase_);
			pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
			vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
			torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
		} break;
		
		default: {
			set_error(ERROR_INVALID_INPUT_MODE);
			return 0;
		}
	}
	
	// Never command a setpoint beyond its limit
	if(ctrl_config.enable_vel_limit) vel_setpoint_ = clamp(vel_setpoint_, -ctrl_config.vel_limit, ctrl_config.vel_limit);
	
	const float Tlim = motor_max_available_torque();   //max_torque = effective_current_lim_ * config_.torque_constant = 60*0.04f = 2.4f;
	torque_setpoint_ = clamp(torque_setpoint_, -Tlim, Tlim);
	
	// Position control
	// TODO Decide if we want to use encoder or pll position here
	float gain_scheduling_multiplier = 1.0f;
	float vel_des = vel_setpoint_;
	if (ctrl_config.control_mode >= CONTROL_MODE_POSITION_CONTROL)
	{
		float pos_err;
		if (ctrl_config.circular_setpoints)
		{
			// Keep pos setpoint from drifting
			pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
			// Circular delta
			pos_err = pos_setpoint_ - *pos_estimate_circular;
			pos_err = wrap_pm(pos_err, *pos_wrap);
		}
		else
		{
			pos_err = pos_setpoint_ - *pos_estimate_linear;
		}
		
		vel_des += ctrl_config.pos_gain * pos_err;
		// V-shaped gain shedule based on position error
		float abs_pos_err = fabsf(pos_err);
		if (ctrl_config.enable_gain_scheduling && abs_pos_err <= ctrl_config.gain_scheduling_width) {
			gain_scheduling_multiplier = abs_pos_err / ctrl_config.gain_scheduling_width;
		}
	}
	
	// Velocity limiting
	float vel_lim = ctrl_config.vel_limit;
	if (ctrl_config.enable_vel_limit)
	{
		vel_des = clamp(vel_des, -vel_lim, vel_lim);
	}
	// Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
	if (ctrl_config.enable_overspeed_error)   // 0.0f to disable
	{
		if (fabsf(*vel_estimate) > ctrl_config.vel_limit_tolerance * vel_lim)
		{
			set_error(ERROR_OVERSPEED);
			return 0;
		}
	}
	
	float vel_gain = ctrl_config.vel_gain;
	float vel_integrator_gain = ctrl_config.vel_integrator_gain;
	// Velocity control
	float torque = torque_setpoint_;
	
	float v_err = 0.0f;
	if (ctrl_config.control_mode >= CONTROL_MODE_VELOCITY_CONTROL)
	{
		v_err = vel_des - *vel_estimate;
		torque += (vel_gain * gain_scheduling_multiplier) * v_err;

		// Velocity integral action before limiting
		torque += vel_integrator_torque_;
	}
	
	// Velocity limiting in current mode
	if (ctrl_config.control_mode < CONTROL_MODE_VELOCITY_CONTROL && ctrl_config.enable_torque_mode_vel_limit)
	{
		torque = limitVel(ctrl_config.vel_limit, *vel_estimate, vel_gain, torque);
	}
	
	// Torque limiting
	uint8_t limited = 0;
	if (torque > Tlim)
	{
		limited = 1;
		torque = Tlim;
	}
	if (torque < -Tlim)
	{
		limited = 1;
		torque = -Tlim;
	}
	
	// Velocity integrator (behaviour dependent on limiting)
	if (ctrl_config.control_mode < CONTROL_MODE_VELOCITY_CONTROL)
	{
		// reset integral if not in use
		vel_integrator_torque_ = 0.0f;
	}
	else
	{
		if (limited)
		{
			// TODO make decayfactor configurable
			vel_integrator_torque_ *= 0.99f;
		}
		else
		{
			vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
		}
		// integrator limiting to prevent windup 
		vel_integrator_torque_ = clamp(vel_integrator_torque_, -ctrl_config.vel_integrator_limit, ctrl_config.vel_integrator_limit);
	}
	/*
	float ideal_electrical_power = 0.0f;
	if (motor_config.motor_type != MOTOR_TYPE_GIMBAL)
	{
		ideal_electrical_power = power_ - \
            SQ(Iq_measured_) * 1.5f * motor_config.phase_resistance - \
            SQ(Id_measured_) * 1.5f * motor_config.phase_resistance;
	}
	else
	{
		ideal_electrical_power = power_;
	}
	mechanical_power_ += ctrl_config.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
	electrical_power_ += ctrl_config.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

	// Spinout check
	// If mechanical power is negative (braking) and measured power is positive, something is wrong
	// This indicates that the controller is trying to stop, but torque is being produced.
	// Usually caused by an incorrect encoder offset
	if ((mechanical_power_ < ctrl_config.spinout_mechanical_power_threshold) && (electrical_power_ > ctrl_config.spinout_electrical_power_threshold))
	{
		set_error(ERROR_SPINOUT_DETECTED);
		return 0;
	}
	*/
	torque_output_ = torque;
	
	// TODO: this is inconsistent with the other errors which are sticky.
	// However if we make ERROR_INVALID_ESTIMATE sticky then it will be
	// confusing that a normal sequence of motor calibration + encoder
	// calibration would leave the controller in an error state.
	motor_error &= ~ERROR_INVALID_ESTIMATE;
	
	return 1;
}
/****************************************************************************/



