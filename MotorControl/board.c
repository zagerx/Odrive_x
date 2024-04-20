
#include "MyProject.h"


/****************************************************************************/
#define  CURRENT_SENSE_MIN_VOLT  0.3f
#define  CURRENT_SENSE_MAX_VOLT  3.0f
#define  CURRENT_ADC_LOWER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f)
#define  CURRENT_ADC_UPPER_BOUND  (uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f)
/****************************************************************************/
float  vbus_voltage=12.0f;
Iph_ABC_t  current0;
/****************************************************************************/
float phase_current_from_adcval(uint32_t ADCValue)
{
	if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND)
	{
		set_error(ERROR_CURRENT_SENSE_SATURATION);
		return 0;
	}
	
	int adcval_bal = (int)ADCValue - (1 << 11);
	float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
	float shunt_volt = amp_out_volt * (1/PHASE_CURRENT_GAIN);   //phase_current_rev_gain_ = 1/20倍放大
	float current = shunt_volt * (1/SHUNT_RESISTANCE);          //shunt_conductance_ = 1/0.001采样电阻;
	return current;
}
/****************************************************************************/
void vbus_sense_adc_cb(uint32_t adc_value) 
{
	float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / 4096;
	vbus_voltage = adc_value * voltage_scale;
}
/****************************************************************************/
//在TIM1的更新中断函数中被调用
uint8_t fetch_and_reset_adcs(Iph_ABC_t *current)
{
	uint8_t all_adcs_done = (((ADC1->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC2->SR & ADC_SR_JEOC) == ADC_SR_JEOC) && ((ADC3->SR & ADC_SR_JEOC) == ADC_SR_JEOC));
  if(!all_adcs_done)return 0;
	
	vbus_sense_adc_cb(ADC1->JDR1);
	current->phB = phase_current_from_adcval(ADC2->JDR1);
	current->phC = phase_current_from_adcval(ADC3->JDR1);
	current->phA = -current->phB - current->phC;
	
	ADC1->SR = ~(ADC_SR_JEOC);
	ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_OVR);
	
	return 1;
}
/****************************************************************************/
#define LED_blink   GPIOD->ODR^=(1<<2)  //PD2
/****************************************************************************/
uint32_t time_cntr=0;
volatile uint32_t timestamp_ = 0;
extern  void control_loop_cb(void);  //在main.c文件中

//中断频率16KHz，进入中断的同时触发ADC
void TIM1_UP_TIM10_IRQHandler(void)
{
	uint32_t i;
	
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	timestamp_ += TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);
	
	if(++time_cntr >= 8000)  //0.5s
	{
		time_cntr = 0;
		LED_blink;
	}
	
	uint8_t counting_down = TIM1->CR1 & TIM_CR1_DIR;
	if(!counting_down)   //=0为递增计数,上臂为低下臂为高,此时采样
	{
		sample_now();                        //读取角度
		fetch_and_reset_adcs(&current0);     //电流采样，获得的采样值在current0
		current_meas_cb(timestamp_, &current0);  //传入采样值并运算
		control_loop_cb();
	}
	else
	{
		for(i=0;i<20;i++);                   //延时，同时等待ADC完毕
		fetch_and_reset_adcs(&current0);     //电流采样
		dc_calib_cb(&current0);  //timestamp_ + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1), 
		pwm_update_cb();         //timestamp_ + 3 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1);
	}
}
/****************************************************************************/



