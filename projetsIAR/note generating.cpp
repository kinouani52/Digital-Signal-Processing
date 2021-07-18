//Basic audio generation

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "sound.h"
#include "stm32f4xx_gpio.h"

const unsigned int samplingFrequency = 16000;

//Global variables

unsigned int chn = 0;
bool flag = false;
float note_signal[36] = {0};
int sample = 0;
int idx = 0;
int signal_stored[5][36] = {0};

// FOR sampling freq = 16k
// la3(36 samples); do3(30); mi3(24); sol3(20); la4(18)



//Audio CODEC Constants
const unsigned char AddressCODEC = 0x94;

//Initialization Functions
void initARM();
void initI2C();
void initI2S();

void gpio_freq();
void notesArray();
void playNote();

//Audio CODEC Functions
void initCODEC();
void writeCODEC(unsigned char reg, unsigned char data);
unsigned char readCODEC(unsigned char reg);
void audioCODEC(unsigned short dataL, unsigned short dataR);

int main(void)
{	
  initARM();
  initI2C();
  initI2S();
  initCODEC();
  gpio_freq(); // func to configure the GPIOs--> PA1(la3), PA2(do3), PA3(mi3), PA6(sol3), PA7(la4) 
  notesArray(); // func to build a matriz 5x36 with values of the notes
  
  //Enable Interrupions
  SPI_I2S_ITConfig(SPI3, SPI_I2S_IT_TXE, ENABLE);
  
  STM_EVAL_LEDInit(LED3); // note: la3 and la4
  STM_EVAL_LEDInit(LED4); // note: do3
  STM_EVAL_LEDInit(LED5); // note: mi3
  STM_EVAL_LEDInit(LED6); // note: sol3
   
  
  //Main Loop
  while(true) 
  {
    
 
      
      playNote();  // func to sum note's vectors.        
      
   
    }
  }
//}

void initARM()
{
  //Clock Configuration
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  
  //Interruption Configuration
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
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
	//GPIO Configuration (SPI3: WS=PA.4)
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3);

    //GPIO Configuration (SPI3: MCK=PC.7, SCK=PC.10, SD=PC.12)
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	
	//I2S Configuration
	SPI_I2S_DeInit(SPI3);
	I2S_InitTypeDef I2S_InitStructure;
    I2S_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	I2S_InitStructure.I2S_Standard = I2S_Standard_Phillips;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
	I2S_InitStructure.I2S_AudioFreq = samplingFrequency;
    I2S_InitStructure.I2S_CPOL = I2S_CPOL_Low;
    I2S_Init(SPI3, &I2S_InitStructure);
    I2S_Cmd(SPI3, ENABLE);
	
	RCC_PLLI2SCmd(DISABLE);
    RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
    RCC_PLLI2SCmd(ENABLE);
    RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY);
}

// INITIALIZATION OF GPIOA IN INPUT MODE Pins 5,7,8 and 9 
void gpio_freq()
{
GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
}

// Generate a Matrix [5][36] to store the values of the notes. 
void notesArray()
{

  int ila3 = 0;
  int ido3 = 0;
  int imi3 = 0;
  int isol3 = 0;
  int ila4 = 0;
  
  for (int m = 0; m <= 35; ++m) // filling Matrix with note's amplitudes values.
  {
    signal_stored[0][ila3] = la3[ila3]; 
    ila3++;
    
    signal_stored[1][ido3] = do3[ido3];
    ido3++;
    if (ido3 > 29) ido3 = 0;
    
    signal_stored[2][imi3] = mi3[imi3];
    imi3++;
    if (imi3 > 23) imi3 = 0;
    
    signal_stored[3][isol3] = sol3[isol3];
    isol3++;
    if (isol3 > 19) isol3 = 0;
    
    signal_stored[4][ila4] = la4[ila4];
    ila4++;
    if (ila4 > 17) ila4 = 0;
  }

}

// associating every different note to a Pin and a LED 
//reading the state of it.
// suming the amplitudes of the desired notes into the signal[] array.

void playNote()
{
  sample = 0;
    
  // la3
  if (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_1) == 0) 
  {
    sample = 35;
    
    STM_EVAL_LEDOn(LED3);
    for (int i = 0; i <= sample; ++i)  
    {
      note_signal[i] = note_signal[i] + signal_stored[0][i];
    } 
     GPIO_ToggleBits(GPIOA, GPIO_Pin_1);
  }
  
  //do3
  if (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_2) == 0) 
  {
    STM_EVAL_LEDOn(LED4);
    if (sample == 0) sample = 29;
    
    for (int i = 0; i <= sample; ++i)  
    {
      note_signal[i] = note_signal[i] + signal_stored[1][i];
    }           
  }
  
  //mi3
  if (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_3) == 0) 
  {
    
    STM_EVAL_LEDOn(LED5);
    if (sample == 0) sample = 23;
      
    for (int i = 0; i <= sample; ++i)  
    {
      note_signal[i] += signal_stored[2][i];
    }
  }
  
  //sol3
  if (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_6) == 0) // sol3
  {
    
    STM_EVAL_LEDOn(LED6);
    if (sample == 0) sample = 19;
    
    for (int i = 0; i <= sample; ++i)  
    {
      note_signal[i] += signal_stored[3][i];
    }
  }
  
  //la4
  if (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_7) == 0) // la4
  {
    
    STM_EVAL_LEDOn(LED3);
    if (sample == 0) sample = 17;
 
    for (int i = 0; i <= sample; ++i)  
    {
     note_signal[i] += signal_stored[4][i];
    }
  }
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
	writeCODEC(0x20, 0xD0); //volume: master A
	writeCODEC(0x21, 0xD0);	//volume: master B
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
	//while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
	
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
  void SPI3_IRQHandler()
  {
    if(SPI_GetITStatus(SPI3, SPI_I2S_IT_TXE) != RESET) 
    {

      if ( sample != 0)
      {
        unsigned short y = (unsigned short)note_signal[idx];
        SPI_I2S_SendData(SPI3,y); // sending data
        
        if(chn==0) chn++;
        else { 
          note_signal[idx] = 0;
          idx++; 
          chn = 0;
          if(idx == sample) 
          {
            idx=0;
           
            
          }
        }
      }
      else 
      {
        SPI_I2S_SendData(SPI3,0);

      }
      
      STM_EVAL_LEDOff(LED3);
      STM_EVAL_LEDOff(LED4);
      STM_EVAL_LEDOff(LED5);
      STM_EVAL_LEDOff(LED6);
      
    }
  }
  
#ifdef __cplusplus
}
#endif		