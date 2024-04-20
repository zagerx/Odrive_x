
#include "MyProject.h"


/*****************************************************************************/
AXIS_State_Type  current_state_;
AXIS_CONFIG  axis_config;

extern  uint8_t  usb_sndbuff[128];
/*****************************************************************************/
bool start_closed_loop_control(void)
{
	controller_pos_estimate_circular_src_ = &pos_circular_;
	controller_pos_wrap_src_ = &ctrl_config.circular_setpoint_range;
	controller_pos_estimate_linear_src_ = &pos_estimate_;
	controller_vel_estimate_src_ = &vel_estimate_;
	
	// To avoid any transient on startup, we intialize the setpoint to be the current position
	control_mode_updated();
	input_pos_updated_ = true;
	// Avoid integrator windup issues
	vel_integrator_torque_ = 0.0f;
	
	motor_torque_setpoint_src_ = &torque_output_;
	
	//enable_current_control_src_ = (motor_config.motor_type != MOTOR_TYPE_GIMBAL);
	Idq_setpoint_src_ = &motor_Idq_setpoint_;
	Vdq_setpoint_src_ = &motor_Vdq_setpoint_;
	phase_src_ = &encoder_config.phase_;
	phase_vel_src_ = &encoder_config.phase_vel_;
	motor_phase_vel_src_ = &encoder_config.phase_vel_;
	
	// In sensorless mode the motor is already armed.
	if (!is_armed_)
	{
		arm();
	}
	
	return true;
}
/*****************************************************************************/
bool run_closed_loop_control_loop(void)
{
	start_closed_loop_control();
	while (is_armed_)
	{delay_us(1000);}
	
	disarm();
	return 0;
}
/*****************************************************************************/
void run_state_machine_loop(void)
{
	bool status=0;
	uint32_t  len;
	
	switch(current_state_)
	{
		case AXIS_STATE_MOTOR_CALIBRATION:{
			status = run_calibration();   //1表示成功，0表示失败
			
			printf("motor R=%.4f,L=%.4e\r\n", motor_config.phase_resistance,motor_config.phase_inductance);  //电感值比较小，采用科学计数法
			printf("P=%.4f,I=%.4f\r\n", pi_gains_[0],pi_gains_[1]);   //打印整定后的PI参数
			printf("status=%d,error=%X.\r\n", status,motor_error);    //1,0
			
			len=sprintf((char *)usb_sndbuff, "motor R=%.4f,L=%.4e\r\n", motor_config.phase_resistance,motor_config.phase_inductance);
			usb_send(usb_sndbuff, len);
			len=sprintf((char *)usb_sndbuff, "P=%.4f,I=%.4f\r\n", pi_gains_[0],pi_gains_[1]);    //打印整定后的PI参数
			usb_send(usb_sndbuff, len);
			len=sprintf((char *)usb_sndbuff, "status=%d,error=%X.\r\n", status,motor_error);     //1,0
			usb_send(usb_sndbuff, len);
			
			if(status)current_state_ = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;  //直接进入校准
			else      current_state_ = AXIS_STATE_UNDEFINED;
		} break;
		
		case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
			if(!is_calibrated_)goto invalid_state_label;
			status = run_offset_calibration();
			
			printf("dir=%d,offset=%d,float=%.2f\r\n", encoder_config.direction, encoder_config.phase_offset, encoder_config.phase_offset_float);  //串口2打印
			printf("status=%d,error=%X.\r\n", status,motor_error);   //1,0
			
			len=sprintf((char *)usb_sndbuff, "dir=%d,offset=%d,off_float=%.2f\r\n", encoder_config.direction, encoder_config.phase_offset, encoder_config.phase_offset_float);  //USB打印
			usb_send(usb_sndbuff, len);
			len=sprintf((char *)usb_sndbuff, "status=%d,error=%X\r\n", status,motor_error);
			usb_send(usb_sndbuff, len);
			
			current_state_ = AXIS_STATE_UNDEFINED;
		} break;
		
		case AXIS_STATE_CLOSED_LOOP_CONTROL: {
			if(!is_calibrated_)goto invalid_state_label;
			status = run_closed_loop_control_loop();
		} break;
		
		case AXIS_STATE_IDLE: {
		} break;
		
		case AXIS_STATE_UNDEFINED: {
		} break;
		
		default: {
			invalid_state_label:
			set_error(ERROR_INVALID_STATE);
			printf("invalid state!\r\n");   //串口打印
			len=sprintf((char *)usb_sndbuff, "invalid state!\r\n");  //USB打印
			usb_send(usb_sndbuff, len);
			status = 0;  // this will set the state to idle
		} break;
	}
	
	// If the state failed, go to idle, else advance task chain
	if (!status)
	{
		current_state_ = AXIS_STATE_IDLE;
	}
//	else
//	{
//		//current_state_++;
//		//current_state_ = AXIS_STATE_UNDEFINED;
//	}
}
/*****************************************************************************/



