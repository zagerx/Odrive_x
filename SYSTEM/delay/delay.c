
#include "delay.h"


/******************************************************************************/
uint32_t uwTick;
/******************************************************************************/
//在ODrive中，systick用于FreeRTOS系统调度，所以1ms定时用TIM14
//TIM14的配置在 Firmware\Board\v3\Src\stm32f4xx_hal_timebases_TIM.c 第78行
void tim14_InitTick(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_TRG_COM_TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_TimeBaseInitStructure.TIM_Period = 1000-1;      //1ms
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;       //84分频=1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM14,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM14,ENABLE);
}
/******************************************************************************/
//中断函数在Firmware\Board\v3\Src\stm32f4xx_it.c 第373行
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM14,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM14,TIM_IT_Update); //清除中断标志位
		uwTick++;
	}
}
/******************************************************************************/

