//Basic filters project

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "pdm_filter.h"

#include "FastFourierTransform.h"

//Audio Acquisition Constants
//const unsigned int samplingFrequency = 8000;
const unsigned int samplingFrequency = 16000;
//const unsigned int samplingFrequency = 24000;
//const unsigned int samplingFrequency = 32000;
//const unsigned int samplingFrequency = 48000;

const unsigned int decimationFactor = 64;
const unsigned int frequencyPDM = ((samplingFrequency*decimationFactor)/(16*2));
const unsigned int sizeBufferPDM = ((((samplingFrequency/1000)*decimationFactor)/8)/2);
const unsigned int sizeBufferPCM = (samplingFrequency/1000);

//Audio CODEC Constants
const unsigned char AddressCODEC = 0x94;

//PDM Filter Variables
PDMFilter_InitStruct Filter;
const unsigned short pdmVolume = 50;
const float pdmHPF = 20;
const float pdmLPF = samplingFrequency/2.4;

//Audio Acquisition Variables
unsigned short micBufferPing[sizeBufferPDM];
unsigned short micBufferPong[sizeBufferPDM];
unsigned short *micBufferWT = micBufferPing;
unsigned short *micBufferRD = micBufferPong;
unsigned int micIndex = 0;
bool micFlag = false;

unsigned short dacBufferWT[sizeBufferPCM];
unsigned short dacBufferRD[sizeBufferPCM];
unsigned int dacIndex = 0;
unsigned int dacChannel = 0;

//Initialization Functions
void initARM();
void initUSART();
void initI2C();
void initI2S();
void initPDM();
void initCODEC();

//Audio CODEC Functions
void writeCODEC(unsigned char reg, unsigned char data);
unsigned char readCODEC(unsigned char reg);

//FFT Variables
const unsigned int N = 64;
const unsigned int S = 10;
float *fftBufferRD;
unsigned int pointIndex = 0;
unsigned int byteIndex = 0;

union float2byte { 
	float f;
	unsigned char c[sizeof(float)]; 
} f2b;

int main(void)
{	
	initARM();
	initUSART();
	initI2C();
	initI2S();
	initPDM();
	initCODEC();
	
	//LED Configuration
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
		
	//Enable Interrupions
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_TXE, ENABLE);
	
	//FFT Variables
	FastFourierTransform *FFT = new FastFourierTransform(N);
	float fftBufferWT[N];
	unsigned int fftIndex = 0;
	unsigned int skip = 0;
	bool fftFlag = false;
	
	//Main Loop
	while(true) {
		if(micFlag==true) {
			micFlag = false;
			STM_EVAL_LEDOn(LED4);
			PDM_Filter_64_LSB((uint8_t*)micBufferRD,dacBufferWT,pdmVolume,&Filter);
			STM_EVAL_LEDOff(LED4);
			dacIndex = 0;

			for(unsigned int i=0;i<sizeBufferPCM;i++) {
				float x = (float)((short int)dacBufferWT[i]);
				//////////////////////////////////////////////////////////
				float y = x;	//bypass audio sample
				fftBufferWT[fftIndex++] = x/32768;
				//////////////////////////////////////////////////////////
				dacBufferRD[i] =  (unsigned short)((short int)y);
			}
			
			if(fftIndex==N) {
				fftIndex = 0;
				pointIndex = 0;
				byteIndex = 0;
				STM_EVAL_LEDOn(LED3);
				fftBufferRD = FFT->fft(fftBufferWT);
				STM_EVAL_LEDOff(LED3);
				if(fftFlag==true) {
					skip++;
					if(skip==S) {
						USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
						STM_EVAL_LEDOn(LED6);
						skip = 0;
					}
				}
			}
		}
		
		if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
			unsigned int n = USART_ReceiveData(USART2);
			switch(n) {
				case 0x00: {
					fftFlag = true;
				} break;
				case 0x01: {
					fftFlag = false;
				} break;
			}
		}
	}
}

void initARM()
{
	//Clock Configuration
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	
	//Interruption Configuration
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	//GPIO Configuration (CODEC.Reset=PD.4)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void initUSART()
{
	//GPIO Configuration (USART2: TX=PA.2, RX=PA.3)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART Configuration
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2,ENABLE);
}

void initI2C()
{
	//GPIO Configuration (I2C1: SDA=PB.9, SCL=PB.6)
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Fast_Speed;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
	
	//I2C Configuration
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

void initI2S()
{
	SPI_I2S_DeInit(SPI2);
	
	//GPIO Configuration (SPI2: SCK=PB.10, MOSI=PC.3)
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
	
	//I2S Configuration (microphone)
	I2S_InitTypeDef I2S_InitStructure;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	I2S_InitStructure.I2S_AudioFreq = frequencyPDM;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_Init(SPI2, &I2S_InitStructure);
	I2S_Cmd(SPI2, ENABLE);
	
	//Initialize CODEC I2S
	SPI_I2S_DeInit(SPI3);
	
	//GPIO Configuration (SPI3: WS=PA.4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);

    //GPIO Configuration (SPI3: MCK=PC.7, SCK=PC.10, SD=PC.12)
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	//I2S Configuration (CODEC)
    I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitStructure.I2S_AudioFreq = samplingFrequency;
    I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
    I2S_Init(SPI3, &I2S_InitStructure);
    I2S_Cmd(SPI3, ENABLE);
	
	//RCC_PLLI2SCmd(DISABLE);
    RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
    RCC_PLLI2SCmd(ENABLE);
    RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY);
}

void initPDM()
{
    Filter.LP_HZ = pdmLPF;
    Filter.HP_HZ = pdmHPF;
    Filter.Fs = samplingFrequency;
    Filter.Out_MicChannels = 1;
    Filter.In_MicChannels = 1;
	PDM_Filter_Init(&Filter);
}

void initCODEC()
{
	//CS43L22 Power-Up Sequence (p.31)
	GPIO_SetBits(GPIOD, GPIO_Pin_4);

    //CS43L22 Initialization Settings (p.32)
    writeCODEC(0x00, 0x99);
    writeCODEC(0x47, 0x80);
    unsigned char n1 = readCODEC(0x32);
    writeCODEC(0x32, n1 | 0x80);
    unsigned char n2 = readCODEC(0x32);
    writeCODEC(0x32, n2 & 0x7F);
    writeCODEC(0x00, 0x00);

    writeCODEC(0x02, 0x01);	//power down
	writeCODEC(0x04, 0xAF);	//HP:on, SPK:off
    writeCODEC(0x05, 0x80);
    writeCODEC(0x06, 0x07);
	writeCODEC(0x20, 0xF0); //volume: master A
	writeCODEC(0x21, 0xF0);	//volume: master B
    writeCODEC(0x02, 0x9E);	//power up
}

void writeCODEC(unsigned char reg, unsigned char data)
{
	//Wait while bus is busy
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	
	//Start the config sequence
	I2C_GenerateSTART(I2C1, ENABLE);
	
	//Test on EV5
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//Transmit the slave address and enable writing operation
	I2C_Send7bitAddress(I2C1, AddressCODEC, I2C_Direction_Transmitter);
	
	//Test on EV6
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);
	
	//Transmit the first address for write operation
	I2C_SendData(I2C1, reg);
	
	//Test on EV8
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
	
	//Prepare the register value to be sent
	I2C_SendData(I2C1, data);
	
	//Test on EV8
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);
	
	//End the configuration sequence
  	I2C_GenerateSTOP(I2C1, ENABLE);
}

unsigned char readCODEC(unsigned char reg)
{
	//Wait while bus is busy
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
	
	//Start the config sequence
 	I2C_GenerateSTART(I2C1, ENABLE);
	
	//Test on EV5
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//Transmit the slave address and enable writing operation
	I2C_Send7bitAddress(I2C1, AddressCODEC, I2C_Direction_Transmitter);
	
	//Test on EV6
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);
	
	//Transmit the register address to be read
	I2C_SendData(I2C1, reg);
	
	//Test on EV8
	while(I2C_GetFlagStatus(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == RESET);
	
	//Send START condition a second time
	I2C_GenerateSTART(I2C1, ENABLE);
	
	//Test on EV5
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);
	
	//Send codec address for read
	I2C_Send7bitAddress(I2C1, AddressCODEC, I2C_Direction_Receiver);  
	
	//Test on EV6
	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR);
	
	///Disable Acknowledgment
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	
	//Test on EV7
 	while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) == ERROR);
	
	//Send STOP Condition
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	//Wait for the byte to be received
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
	
	//Read the byte received from the codec
	unsigned char data = I2C_ReceiveData(I2C1);
	
	//Wait for STOP flag to be cleared
	while(I2C1->CR1 & I2C_CR1_STOP);
	
	//Re-Enable Acknowledgment to be ready for another reception
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	//Clear AF flag for next communication
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);
	
	return(data);
}

#ifdef __cplusplus
extern "C" {
#endif
void SPI2_IRQHandler()
{
	if(SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET) {
		unsigned short sample = SPI_I2S_ReceiveData(SPI2);
		micBufferWT[micIndex++] = HTONS(sample);
		if(micIndex==sizeBufferPDM) {
			unsigned short *tmp = micBufferWT;
			micBufferWT = micBufferRD;
			micBufferRD = tmp;
			micFlag = true;
			micIndex = 0;
		}
	}
}

void SPI3_IRQHandler()
{
	if(SPI_GetITStatus(SPI3, SPI_I2S_IT_TXE) != RESET) {
		unsigned short x = dacBufferRD[dacIndex];
		SPI_I2S_SendData(SPI3,x);
		
		if(dacChannel==0) dacChannel++;
		else { 
			dacIndex++;
			dacChannel = 0;
		}
	}
}

void USART2_IRQHandler()
{
	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
		if(pointIndex<N/2) {
			if(byteIndex==0) f2b.f = fftBufferRD[pointIndex];
			USART_SendData(USART2,f2b.c[byteIndex++]);
			if(byteIndex==4) {
				byteIndex=0;
				pointIndex++;
			}
		}
		else {
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
			STM_EVAL_LEDOff(LED6);
		}
	}
}

#ifdef __cplusplus
}
#endif