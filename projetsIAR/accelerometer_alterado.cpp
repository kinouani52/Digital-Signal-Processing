//Basic LIS3DSH accelerometer code

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

//Initialization Functions
void initARM();
void initUSART();
void initSPI();
void initMEMS();

//Accererometer Functions
void writeMEMS(unsigned char reg, unsigned char data);
unsigned char readMEMS(unsigned char reg);
float getX();
float getY();
float getZ();


//Global Variables
bool flag = false;

// coef. of IIR filter n = 4
      
const float b[] = {0.00018321602336960943, 0.00073286409347843773, 0.0010992961402176565, 0.00073286409347843773,  0.00018321602336960943};
const float a[] = {1, -3.3440678377118731, 4.2388639508840642, -2.4093428565863175, 0.51747819978804011};

float xx[5] = {0,0,0,0,0};    // Array inputs axis X
float yx[5] = {0,0,0,0,0};    // Array outputs axis X

float xy[5] = {0,0,0,0,0};    // Array inputs axis Y
float yy[5] = {0,0,0,0,0};    // Array outputs axis Y

float xz[5] = {0,0,0,0,0};    // Array inputs axis Z
float yz[5] = {0,0,0,0,0};    // Aarray outputs axis Z

int main(void)
{	
  initARM();
  initUSART();
  initSPI();
  initMEMS();
  
  //LED Configuration
  STM_EVAL_LEDInit(LED6);
  
  
  while(true) {
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
      unsigned int n = USART_ReceiveData(USART1);
      switch(n) {
      case 0x00: {
        flag = true;
        STM_EVAL_LEDOn(LED6);
      } break;
      case 0x01: {
        flag = false;
        STM_EVAL_LEDOff(LED6);
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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  //External Interruption
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);
  
  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  //Interruption Configuration
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void initUSART()
{
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
}

void initSPI()
{
  //GPIO Configuration (SPI1: SCK=PA.5, MISO=PA.6, MOSI=PA.7, CS=PE.3, INT1=PE.0, INT2=PE.1)
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  
  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_Init(GPIOE, &GPIO_InitStructure);*/
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
  
  //SPI Configuration
  SPI_I2S_DeInit(SPI1);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
}

void initMEMS()
{
  //CTRL_REG3 (23h) pp.32
  writeMEMS(0x23,0xE8);	//INT1 = enable (pulsed, high)
  
  //CTRL_REG4 (20h) pp.33
  //writeMEMS(0x20,0x37);	//fs = 12.5Hz
  writeMEMS(0x20,0x47);	//fs = 25Hz
  //writeMEMS(0x20,0x57);	//fs = 50Hz
  //writeMEMS(0x20,0x67);	//fs = 100Hz
  
  //CTRL_REG5 (24h) pp.34
  writeMEMS(0x24,0x00);	// scale = [-2G +2G]
}

void writeMEMS(unsigned char reg, unsigned char data)
{
  //Set CS low
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  
  //Send register address
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, reg);
  
  //Read dummy byte
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI1);
  
  //Send register value
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, data);
  
  //Read dummy byte
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI1);
  
  //Set CS high
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

unsigned char readMEMS(unsigned char reg)
{
  //Set CS low
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  
  //Send register address
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, reg | 0x80);
  
  //Read dummy byte
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  SPI_I2S_ReceiveData(SPI1);
  
  //Send dummy byte
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_SendData(SPI1, 0x00);
  
  //Receive register value
  while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
  unsigned char data = SPI_I2S_ReceiveData(SPI1);
  
  //Set CS high
  GPIO_SetBits(GPIOE, GPIO_Pin_3);
  
  return(data);
}

float getX()
{       
  //OUT_X_L (28h) pp.39
  unsigned short x1 = readMEMS(0x28);   // Storing a 8 bits INPUT in x1 (16bits)
  
  //OUT_X_L (29h) pp.39
  unsigned short x2 = readMEMS(0x29);    // Storing a 8 bits INPUT in x2 (16bits)
  
  short int x_aux = (x2<<8)|x1;         // joing together x2 and x1 into 16 bits integer
  
  xx[0] = x_aux;                 // add 16 bits INPUT to the INPUT array
  yx[0] = 0;
  
  for (int i = 1; i <= 4; i++)           // Calculating the next OUTPUT based on olders INPUTS and OUTPUTS
  {
    yx[0] += b[i] * xx[i] - a[i] * yx[i];       // Y = E(b*x) - E(b*y)
  }
  
  yx[0] += b[0] * xx[0];                        // Y = E(b*x) - E(b*y)                
  
  for (int i = 3; i >= 0; i--)          //   Storing values "one position below"    
  {
    xx[i+1] = xx[i];
    yx[i+1] = yx[i];
  }
  
  return(yx[0]);                // returning 16 bits last OUTPUT of X axis 
}

float getY()
{
  //OUT_Y_L (2Ah) pp.39
  unsigned short y1 = readMEMS(0x2A);   // Storing a 8 bits INPUT in y1 (16bits)
  
  //OUT_Y_L (2Bh) pp.39
  unsigned short y2 = readMEMS(0x2B);   // Storing a 8 bits INPUT in y2 (16bits)
  
  short int y_aux = (y2<<8)|y1;         // joing together y2 and y1 into 16 bits integer
  
  xy[0] = y_aux;         // add 16 bits INPUT to the INPUT array
  yy[0] = 0;
  
  for (int i = 1; i <= 4; i++)          // Calculating the next OUTPUT based on olders INPUTS and OUTPUTS
  {
    yy[0] += b[i] * xy[i] - a[i] * yy[i];       // Y = E(b*x) - E(b*y)
  }
  
  yy[0] += b[0] * xy[0];                // Y = E(b*x) - E(b*y)
  
  for (int i = 3; i >= 0; i--)          //   Storing values "one position below"
  {
    xy[i+1] = xy[i];
    yy[i+1] = yy[i];
  }
  
  return(yy[0]);                // returning 16 bits last OUTPUT of Y axis
}

float getZ()
{
  //OUT_Z_L (2Ch) pp.39
  unsigned short z1 = readMEMS(0x2C);    // Storing a 8 bits INPUT in z1 (16bits)
  
  //OUT_Z_L (2Dh) pp.39
  unsigned short z2 = readMEMS(0x2D);    // Storing a 8 bits INPUT in z2 (16bits)
  
  short int z_aux = (z2<<8)|z1;         // joing together z2 and z1 into 16 bits integer
  
  xz[0] = z_aux;                // add 16 bits INPUT to the INPUT array
  yz[0] = 0;
  
  for (int i = 1; i <= 4; i++)          // Calculating the next OUTPUT based on olders INPUTS and OUTPUTS
  {
    yz[0] += b[i] * xz[i] - a[i] * yz[i];       // Y = E(b*x) - E(b*y)
  }
  
  yz[0] += b[0] * xz[0];                // Y = E(b*x) - E(b*y)
  
  for (int i = 3; i >= 0; i--)          //   Storing vecto's values "one position below"
  {
    xz[i+1] = xz[i];
    yz[i+1] = yz[i];
  }
  
  return(yz[0]);                // returning 16 bits last OUTPUT of Z axis
}

#ifdef __cplusplus
extern "C" {
#endif
  void EXTI0_IRQHandler()
  {
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
      getX();  
      getY();
      getZ();
      unsigned char x1 = (short int)yx[0]&0xFF; //  shifting yx[0] bits (8bits)
      unsigned char x2 = (short int)yx[0]>>8;   //  shifting yx[0] bits (8bits)
      unsigned char y1 = (short int)yy[0]&0xFF; //  shifting yy[0] bits (8bits)
      unsigned char y2 = (short int)yy[0]>>8;   //  shifting yy[0] bits (8bits)
      unsigned char z1 = (short int)yz[0]&0xFF; //  shifting yz[0] bits (8bits)
      unsigned char z2 = (short int)yz[0]>>8;   //  shifting yz[0] bits (8bits)
      
      
      if(flag==true) 
      {
        
        USART_SendData(USART1,x1);           // sending 8 bits of filtered signal of X axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        
        
        
        USART_SendData(USART1,x2);           // sending 8 bits of filtered signal of X axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        
        
        USART_SendData(USART1,y1);           // sending 8 bits of filtered signal of Y axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        
        
        
        USART_SendData(USART1,y2);           // sending 8 bits of filtered signal of Y axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        
        
        USART_SendData(USART1,z1);           // sending 8 bits of filtered signal of Z axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        
        
        
        USART_SendData(USART1,z2);           // sending 8 bits of filtered signal of Z axis
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
      }
      
      EXTI_ClearITPendingBit(EXTI_Line0);
    }
  }
#ifdef __cplusplus
}
#endif