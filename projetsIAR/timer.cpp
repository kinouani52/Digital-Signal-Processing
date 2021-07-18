//Basic Timer & IRQ program

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

int main(void)
{
	//Clock Configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	//Interruption Configuration
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	//LED Configuration
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	//Timer Configuration
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(SystemCoreClock/1000)-1;	// -> kHz
	TIM_TimeBaseStructure.TIM_Period = 2000-1;									// -> Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 1000-1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 500-1;
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 250-1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

	//Enable interrupts
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);

	//Enable timer
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM5, ENABLE);

	//Main Loop
	while(true);
}

#ifdef __cplusplus
extern "C" {
#endif
void TIM2_IRQHandler()
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		STM_EVAL_LEDToggle(LED3);
	}
}

void TIM3_IRQHandler()
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		STM_EVAL_LEDToggle(LED4);
	}
}

void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		STM_EVAL_LEDToggle(LED5);
	}
}

void TIM5_IRQHandler()
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		STM_EVAL_LEDToggle(LED6);
	}
}
#ifdef __cplusplus
}
#endif