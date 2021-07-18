//Serial Port (USART) basic program

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

int main(void)
{
	//Clock Configuration
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//LED Configuration
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	
	//Push-Button Configuration
	STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);
	
	//GPIO Configuration (USART1: TX=PB.6, RX=PB.7)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//USART Configuration
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1,ENABLE);
	
	unsigned int k = 48;

	//Main Loop
	while(true) {
    	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
			unsigned int n = USART_ReceiveData(USART1);

			switch(n) {
				case '0': STM_EVAL_LEDOn(LED3); break;
				case '1': STM_EVAL_LEDOn(LED4); break;
				case '2': STM_EVAL_LEDOn(LED5); break;
				case '3': STM_EVAL_LEDOn(LED6); break;
				case '4': STM_EVAL_LEDOff(LED3); break;
				case '5': STM_EVAL_LEDOff(LED4); break;
				case '6': STM_EVAL_LEDOff(LED5); break;
				case '7': STM_EVAL_LEDOff(LED6); break;
				default: USART_SendData(USART1,'X');
			}
		}

		unsigned int b = STM_EVAL_PBGetState(BUTTON_USER);
		if(b==1) {
			//USART_SendData(USART1,'K');
			USART_SendData(USART1,k);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		  
			k++;
			if(k==58) k=65;
			if(k==91) k=97;
			if(k==123) k=48;
		}
	}
	
}

