
#include "MyProject.h"


/*****************************************************************************/
//GPIO1为CS
#define  SPI_CS0_L   GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define  SPI_CS0_H   GPIO_SetBits(GPIOA, GPIO_Pin_0)

#define  SPI3_TX_OFF() {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=0<<(12*2);}  //PC12(MOSI)输入浮空
#define  SPI3_TX_ON()  {GPIOC->MODER&=~(3<<(12*2));GPIOC->MODER|=2<<(12*2);}  //PC12(MOSI)复用推挽输出
/*****************************************************************************/
ENCODER_CONFIG   encoder_config;

bool index_found_ = false;   //针对ABZ编码器，Z信号
bool is_ready_ = false;      //编码器是否准备就绪。电机上电校准后为true。
int32_t shadow_count_ = 0;   //编码器累计计数。
int32_t count_in_cpr_ = 0;   //编码器当前计数值。
float interpolation_ = 0.0f; //编码器当前插补值。
float pos_estimate_counts_ = 0.0f;  //当前估算的位置值，单位[count]   
float pos_cpr_counts_ = 0.0f;       //当前约束在cpr范围内的位置值，单位[count]
//float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
float vel_estimate_counts_ = 0.0f;  //当前估算转速，单位[count/s]
float pll_kp_ = 0.0f;   // [count/s / count]
float pll_ki_ = 0.0f;   // [(count/s^2) / count]
float calib_scan_response_ = 0.0f; // debug report from offset calib
int32_t  pos_abs_ = 0;  //绝对值编码器的位置
float spi_error_rate_ = 0.0f;

float pos_estimate_ = 0.0f; //当前估算的位置值，单位[turn]
float vel_estimate_ = 0.0f; //当前估算转速，单位[turn/s]
float pos_circular_ = 0.0f; //环形位置模式下当前位置值，单位[turn]

bool pos_estimate_valid_ = false;   //位置估算是否可用
bool vel_estimate_valid_ = false;   //速度估算是否可用

int16_t  tim_cnt_sample_;

bool abs_spi_pos_updated_ = false;  //绝对值编码器角度是否被正确读出
/*****************************************************************************/
/*****************************************************************************/
void encoder_set_error(uint32_t error) 
{
	vel_estimate_valid_ = false;
	pos_estimate_valid_ = false;
	set_error(error);
}
/*****************************************************************************/
void update_pll_gains(void)
{
	pll_kp_ = 2.0f * encoder_config.bandwidth;  // basic conversion to discrete time
	pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped
	// Check that we don't get problems with discrete time approximation
	if (!(current_meas_period * pll_kp_ < 1.0f))
		encoder_set_error(ERROR_UNSTABLE_GAIN);
}
/*****************************************************************************/

/*****************************************************************************/
//初始化三种SPI接口的编码器的参数, 初始化I2C接口或者SPI接口
void MagneticSensor_Init(void)
{
	//读取flash内保存的参数
	encoder_config.mode = ENCODER_mode;  //选择编码器型号，参数设置宏定义在MyProject.h文件中
	encoder_config.cpr = ENCODER_cpr;    //编码器cpr
	encoder_config.bandwidth = ENCODER_bandwidth;   //编码器带宽
	encoder_config.calib_range = 0.02f; // Accuracy required to pass encoder cpr check  2%误差
	encoder_config.calib_scan_distance = 16.0f * M_PI; // rad electrical    校准的时候正反转8个极对数
	encoder_config.calib_scan_omega = 4.0f * M_PI;     // rad/s electrical  转速2个极对数/秒，所以正转4秒，反转再4秒
	encoder_config.phase_offset = 0;        // Offset between encoder count and rotor electrical phase
	encoder_config.phase_offset_float = 0.0f; // Sub-count phase alignment offset
	encoder_config.index_offset = 0.0f;
	encoder_config.use_index = false;
	encoder_config.pre_calibrated = false;
	encoder_config.direction = 0; // direction with respect to motor
	encoder_config.use_index_offset = true;
	encoder_config.enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
	encoder_config.find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
	
	update_pll_gains();    //锁相环参数整定
	
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL:
			TIM3_Encoder_Init();         //ABZ
			break;
		case MODE_SPI_AS5047P:
			SPI3_Init_(SPI_CPOL_Low);    //AS5047P
			break;
		case MODE_SPI_MT6701:
			SPI3_Init_(SPI_CPOL_Low);    //MT6701
			break;
		case MODE_SPI_MA730:
			SPI3_Init_(SPI_CPOL_High);   //MA730
			break;
		case MODE_SPI_TLE5012B:        //TLE5012B
			SPI3_Init_(SPI_CPOL_Low);
			break;
	}
}
/*****************************************************************************/
// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool run_offset_calibration(void)
{
	uint32_t  i;
	const float start_lock_duration = 1.0f;
	
	// Require index found if enabled
	if (encoder_config.use_index && !index_found_)
	{
		encoder_set_error(ERROR_INDEX_NOT_FOUND_YET);
		return false;
	}
	
	// We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
	// Therefore we have to sync them for calibration
	shadow_count_ = count_in_cpr_;
	
	// Reset state variables
	memset(&openloop_controller_,0, sizeof(OPENLOOP_struct));   //清零openloop的结构体

	
	float max_current_ramp = motor_config.calibration_current / start_lock_duration * 2.0f;
	openloop_controller_.max_current_ramp_ = max_current_ramp;
	openloop_controller_.max_voltage_ramp_ = max_current_ramp;
	openloop_controller_.max_phase_vel_ramp_ = INFINITY;
	openloop_controller_.target_current_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? motor_config.calibration_current : 0.0f;   //功率电机，校准电流为目标电流
	openloop_controller_.target_voltage_ = motor_config.motor_type != MOTOR_TYPE_GIMBAL ? 0.0f : motor_config.calibration_current;   //云台电机，校准电流为目标电压
	openloop_controller_.target_vel_ = 0.0f;
	openloop_controller_.total_distance_ = 0.0f;
	openloop_controller_.phase_ = wrap_pm_pi(0 - encoder_config.calib_scan_distance / 2.0f);
	
	//enable_current_control_src_ = (motor_config.motor_type != MOTOR_TYPE_GIMBAL);
	Idq_setpoint_src_ = &openloop_controller_.Idq_setpoint_;   //指针指向
	Vdq_setpoint_src_ = &openloop_controller_.Vdq_setpoint_;
	phase_src_ = &openloop_controller_.phase_;
	phase_vel_src_ = &openloop_controller_.phase_vel_;
	motor_phase_vel_src_ = &openloop_controller_.phase_vel_;
	
	arm();
	// go to start position of forward scan for start_lock_duration to get ready to scan
	for (i=0; i<1000; i++)   //电机定位1秒钟，电角度0°
	{
		if (!is_armed_)return false; // TODO: return "disarmed" error code
		delay_us(1000);   //1ms
	}
	
	int32_t init_enc_val = shadow_count_;   //读取当前角度。shadow_count_在encoder_update()函数中读角度时更新
	uint32_t num_steps = 0;
	int64_t encvaluesum = 0;
	
	openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;   //设置正转速度 4Pi/s
	openloop_controller_.total_distance_ = 0.0f;
	
	// scan forward
	while (is_armed_)
	{
		if(openloop_controller_.total_distance_ >= encoder_config.calib_scan_distance)break;   //转过16Pi（8个极对数）
		encvaluesum += shadow_count_;   //累加当前机械角度值
		num_steps++;      //大约转了4秒，转完约等于4000。    开环控制启动和停止都需要时间，所以会有误差
		delay_us(1000);   //1ms
	}
	
	// Check response and direction
	if (shadow_count_ > init_enc_val + 8)encoder_config.direction = 1;        //当前角度比初始角度大就是正转
	else if (shadow_count_ < init_enc_val - 8)encoder_config.direction = -1;  //否则就是反转
	else
	{
		encoder_set_error(ERROR_NO_RESPONSE);   //编码器接线不好，或者配置型号不对
		disarm();
		return false;
	}
	
	// Check CPR
	float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
	float expected_encoder_delta = encoder_config.calib_scan_distance / elec_rad_per_enc;   //理论上的角度差值
	calib_scan_response_ = fabsf(shadow_count_ - init_enc_val);                             //实际的角度差值
	if (fabsf(calib_scan_response_ - expected_encoder_delta) / expected_encoder_delta > encoder_config.calib_range)  //误差率大于2%，认为错误
	{
		encoder_set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
		disarm();
		return false;
	}
	
	openloop_controller_.target_vel_ = -encoder_config.calib_scan_omega;  //设置反转速度 -4Pi/s
	
	// scan backwards
	while (is_armed_)
	{
		if(openloop_controller_.total_distance_ <= 0.0f)break;   //转过16Pi（8个极对数）
		encvaluesum += shadow_count_;   //累加当前机械角度值
		num_steps++;      //大概转了4秒钟，4000。 num_steps此时约为8000
		delay_us(1000);   //1ms
	}
	
	// Motor disarmed because of an error
	if (!is_armed_)return false;
	
	disarm();
	
	encoder_config.phase_offset = encvaluesum / num_steps;   //累加后的角度/累加次数=中间值，比如1——100累加=5050 /100=50.5。在这里，phase_offset就是电角度为8Pi时对应的角度
	int32_t residual = encvaluesum - ((int64_t)encoder_config.phase_offset * (int64_t)num_steps);
	encoder_config.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase  其实用不了这么高的精度，小数部分可有可无
	
	is_ready_ = true;
	return true;
}
/*****************************************************************************/
/*****************************************************************************/
uint8_t ams_parity(uint16_t v)
{
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}
/*****************************************************************************/
void abs_spi_cb(void)
{
	uint16_t rawVal;
	uint16_t pos;
	
	switch(encoder_config.mode)
	{
		case MODE_SPI_AS5047P:
			SPI_CS0_L;
			rawVal = SPIx_ReadWriteByte(0xffff);  //encoder.hpp 第144行
			SPI_CS0_H;
		
			if(ams_parity(rawVal) || ((rawVal >> 14) & 1))
				return;
			pos = (rawVal & 0x3fff);
			break;
		case MODE_SPI_MA730:
		case MODE_SPI_MT6701:
			SPI_CS0_L;
			rawVal = SPIx_ReadWriteByte(0);
			SPI_CS0_H;
			pos = (rawVal >> 2) & 0x3fff;
			break;
		case MODE_SPI_TLE5012B: {
			SPI_CS0_L;
			SPIx_ReadWriteByte(0x8020);
			SPI3_TX_OFF();
			__nop();__nop();__nop();__nop();//__nop();__nop();__nop();__nop();  //Twr_delay=130ns min，实际测试不加延时也可以
			rawVal = SPIx_ReadWriteByte(0xffff);
			SPI_CS0_H;
			SPI3_TX_ON();
			pos = (rawVal & 0x7fff);
		} break;
		case MODE_INCREMENTAL:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			break;
	}
	
	pos_abs_ = pos;
	abs_spi_pos_updated_ = 1;
	if (encoder_config.pre_calibrated) is_ready_ = 1;
}
/*****************************************************************************/
void sample_now(void)
{
	switch(encoder_config.mode)
	{
		case MODE_INCREMENTAL:
			tim_cnt_sample_ = TIM3->CNT;
			break;
		case MODE_SPI_AS5047P:
		case MODE_SPI_MT6701:
		case MODE_SPI_MA730:
		case MODE_SPI_TLE5012B:
			abs_spi_cb();
			break;
	}
}
/****************************************************************************/
bool encoder_update(void)
{
	// update internal encoder state.
	int32_t delta_enc = 0;
	int32_t pos_abs_latched = pos_abs_; //LATCH
	sample_now();                        //跟新角度RAW

	switch(encoder_config.mode)
	{

		
		case MODE_SPI_AS5047P:
		 {
			abs_spi_pos_updated_ = false;
			delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
			delta_enc = mod(delta_enc, encoder_config.cpr);
			if(delta_enc > encoder_config.cpr/2)
				delta_enc -= encoder_config.cpr;
		} break;
		
		default:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			return 0;
	}
	
	shadow_count_ += delta_enc;
	count_in_cpr_ += delta_enc;
	count_in_cpr_ = mod(count_in_cpr_, encoder_config.cpr);
	
	if((encoder_config.mode==MODE_SPI_AS5047P)||(encoder_config.mode==MODE_SPI_MT6701)||(encoder_config.mode==MODE_SPI_MA730))
		count_in_cpr_ = pos_abs_latched;
	
	// Memory for pos_circular
	float pos_cpr_counts_last = pos_cpr_counts_;
	
	// run pll (for now pll is in units of encoder counts)
	// Predict current pos
	pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
	pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
	
	// discrete phase detector
	float delta_pos_counts = (float)(shadow_count_ - (int32_t)pos_estimate_counts_);
	float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)pos_cpr_counts_);
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(encoder_config.cpr));
	//delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
	// pll feedback
	pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
	pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
	pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(encoder_config.cpr));
	vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
	uint8_t snap_to_zero_vel = false;
	if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_)
	{
		vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
		snap_to_zero_vel = true;
	}
	
	// Outputs from Encoder for Controller
	pos_estimate_ = pos_estimate_counts_ / (float)encoder_config.cpr;
	vel_estimate_ = vel_estimate_counts_ / (float)encoder_config.cpr;
	
	// TODO: we should strictly require that this value is from the previous iteration
	// to avoid spinout scenarios. However that requires a proper way to reset
	// the encoder from error states.
	float pos_circular = pos_circular_;  //.any().value_or(0.0f);
	pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)encoder_config.cpr, 1.0f);
	pos_circular = fmodf_pos(pos_circular, ctrl_config.circular_setpoint_range);    //单圈循环模式
	pos_circular_ = pos_circular;
	
	//// run encoder count interpolation
	int32_t corrected_enc = count_in_cpr_ - encoder_config.phase_offset;
	// if we are stopped, make sure we don't randomly drift
	if(snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
	{
		interpolation_ = 0.5f;
		// reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
	}
	else if(delta_enc > 0){
		interpolation_ = 0.0f;
	}else if(delta_enc < 0){
		interpolation_ = 1.0f;
	}
	else {
		// Interpolate (predict) between encoder counts using vel_estimate,
		interpolation_ += current_meas_period * vel_estimate_counts_;
		// don't allow interpolation indicated position outside of [enc, enc+1)
		if (interpolation_ > 1.0f) interpolation_ = 1.0f;
		if (interpolation_ < 0.0f) interpolation_ = 0.0f;
	}
	float interpolated_enc = corrected_enc + interpolation_;
	
	//// compute electrical phase
	//TODO avoid recomputing elec_rad_per_enc every time
	float elec_rad_per_enc = motor_config.pole_pairs * 2 * M_PI * (1.0f / (float)(encoder_config.cpr));
	float ph = elec_rad_per_enc * (interpolated_enc - encoder_config.phase_offset_float);
	
	if(is_ready_)   //校准完成后才能得到角度
	{
		encoder_config.phase_ = wrap_pm_pi(ph) * encoder_config.direction;
		encoder_config.phase_vel_ = (2*M_PI) * vel_estimate_ * motor_config.pole_pairs * encoder_config.direction;
	}
	
	return 1;
}
/****************************************************************************/



