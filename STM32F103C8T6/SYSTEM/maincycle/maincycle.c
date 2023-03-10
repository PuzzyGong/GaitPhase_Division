#include "maincycle.h"

static uint16_t TIM4_Period = 0;
//
//------------------------------------------------------------------------------------------//
static void tim_init(uint16_t ms)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
	TIM_TimeBaseInitStruct.TIM_Prescaler = 719;
	TIM_TimeBaseInitStruct.TIM_Period = 100 * ms - 1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM4_Period = 100 * ms - 1;
}

static void nvic_init()
{
    NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(& NVIC_InitStruct);
}
void maincycle_ms_init(uint16_t ms)
{
    tim_init(ms);   
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	
	nvic_init();
	TIM_Cmd(TIM4, ENABLE);
}
//------------------------------------------------------------------------------------------//


//
//------------------------------------------------------------------------------------------//
static uint16_t TIM8_Counter = 0;
void CPUoccupationRate_Calculatestart(void)
{
    TIM8_Counter = TIM_GetCounter(TIM4);
}
float CPUoccupationRate_Calculatefinish(void)
{
    return ( TIM_GetCounter(TIM4) - TIM8_Counter ) * 100.0 / TIM4_Period;
}
//------------------------------------------------------------------------------------------//
