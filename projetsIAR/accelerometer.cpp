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
float filter[201]= {-7.5906e-05,           -0.00022365,           -0.00025479,           -0.00014819,           4.5715e-05,           0.0002272,           0.00029625,           0.00020557,           -9.2905e-06,           -0.00024098,           -0.00036136,           -0.00028986,           -4.4134e-05,           0.0002575,           0.00044989,           0.00040859,           0.00012646,           -0.00026635,           -0.00055833,           -0.00056738,           -0.00025002,           0.00025442,           0.00067958,           0.00076915,           0.00042686,           -0.00020636,           -0.00080281,           -0.0010136,           -0.0006679,           0.00010506,           0.00091341,           0.0012964,           0.00098221,           6.7762e-05,           -0.00099314,           -0.0016092,           -0.0013762,           -0.00033086,           0.0010203,           0.0019391,           0.0018533,           0.00070292,           -0.00097019,           -0.0022682,           -0.0024129,           -0.0012021,           0.00081492,           0.0025742,           0.0030507,           0.0018458,           -0.00052378,           -0.0028293,           -0.0037582,           -0.0026507,           6.2688e-05,           0.0030008,           0.004523,           0.0036338,           0.00060699,           -0.0030494,           -0.0053291,           -0.0048141,           -0.001531,           0.0029282,           0.0061572,           0.0062156,           0.0027671,           -0.0025784,           -0.0069859,           -0.0078745,           -0.0043952,           0.0019215,           0.0077919,           0.0098509,           0.0065369,           -0.00084398,           -0.0085516,           -0.012254,           -0.0093972,           -0.00083889,           0.0092416,           0.015298,           0.013363,           0.0034696,           -0.0098402,           -0.019448,           -0.019268,           -0.0077973,           0.010328,           0.025877,           0.029301,           0.015891,           -0.010688,           -0.038503,           -0.051531,           -0.036442,           0.010909,           0.08226,           0.15874,           0.21728,           0.23919,           0.21728,           0.15874,           0.08226,           0.010909,           -0.036442,           -0.051531,           -0.038503,           -0.010688,           0.015891,           0.029301,           0.025877,           0.010328,           -0.0077973,           -0.019268,           -0.019448,           -0.0098402,           0.0034696,           0.013363,           0.015298,           0.0092416,           -0.00083889,           -0.0093972,           -0.012254,           -0.0085516,           -0.00084398,           0.0065369,           0.0098509,           0.0077919,           0.0019215,           -0.0043952,           -0.0078745,           -0.0069859,           -0.0025784,           0.0027671,           0.0062156,           0.0061572,           0.0029282,           -0.001531,           -0.0048141,           -0.0053291,           -0.0030494,           0.00060699,           0.0036338,           0.004523,           0.0030008,           6.2688e-05,           -0.0026507,           -0.0037582,           -0.0028293,           -0.00052378,           0.0018458,           0.0030507,           0.0025742,           0.00081492,           -0.0012021,           -0.0024129,           -0.0022682,           -0.00097019,           0.00070292,           0.0018533,           0.0019391,           0.0010203,           -0.00033086,           -0.0013762,           -0.0016092,           -0.00099314,           6.7762e-05,           0.00098221,           0.0012964,           0.00091341,           0.00010506,           -0.0006679,           -0.0010136,           -0.00080281,           -0.00020636,           0.00042686,           0.00076915,           0.00067958,           0.00025442,           -0.00025002,           -0.00056738,           -0.00055833,           -0.00026635,           0.00012646,           0.00040859,           0.00044989,           0.0002575,           -4.4134e-05,           -0.00028986,           -0.00036136,           -0.00024098,           -9.2905e-06,           0.00020557,           0.00029625,           0.0002272,           4.5715e-05,           -0.00014819,           -0.00025479,           -0.00022365,           -7.5906e-05};

 //float Xnote = 0;
//float Ynote = 0;
//float Znote = 0;

 float Yn[201]={0};
//float Y=0;
float Y[201]={0};

 float Zn[201]={0};
 //float Z=0;
float Z[201]={0};
 
 float Xn[201]={0};   
// float X=0;
 float X[201]={0};
 
 
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
	writeMEMS(0x20,0x37);	//fs = 12.5Hz
	//writeMEMS(0x20,0x47);	//fs = 25Hz
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
	unsigned short x1 = readMEMS(0x28);
	
	//OUT_X_L (29h) pp.39
	unsigned short x2 = readMEMS(0x29);
	
	short int x = (x2<<8)|x1;
        
         int i;
         
             
          Xn[0]=x;
         X[0]=0;
        
          // X=0;
        
              for(i=0; i<201; i++) 
                {
     
                    X[0] += Zn[i]*filter[i];
                      
                }
              for (i=199; i>0; i--)
              {
                Xn[i+1] = Xn[i];
                //X[i+1] = X[i];
              }
         
   
   return  X[0];	
}

float getY()
{
	//OUT_Y_L (2Ah) pp.39
	unsigned short y1 = readMEMS(0x2A);
	
	//OUT_Y_L (2Bh) pp.39
	unsigned short y2 = readMEMS(0x2B);
	
	short int y = (y2<<8)|y1;
        
       Yn[0]=y;
        Y[0]=0;
        
           int i;
        //Y=0;
        
              for(i=0; i<201; i++) 
                {
     
                    Y[0] += Yn[i]*filter[i];
                      
                }
              for (i=199; i>=0; i--)
              {
                Yn[i+1] = Yn[i];
                //Y[i+1] = Y[i];
              }
         
   
   return Y[0];
	
	
}

float getZ()
{
	//OUT_Z_L (2Ch) pp.39
	unsigned short z1 = readMEMS(0x2C);
	
	//OUT_Z_L (2Dh) pp.39
	unsigned short z2 = readMEMS(0x2D);
	
	short int z = (z2<<8)|z1;
	
	   int i;
     
           Zn[0]=z;
          Z[0]=0;
        
         // Z = 0;
        
              for(i=0; i<201; i++) 
                {
     
                    Z[0] += Zn[i]*filter[i];
                      
                }
              for (i=199; i>0; i--)
              {
                Zn[i+1] = Zn[i];
                //Z[i+1] = Z[i];
              }
         
   
   return Z[0];
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
                
		unsigned char x1 =(short int)X[0]&0xFF ;
		unsigned char x2 = (short int)X[0]>>8 ;
		unsigned char y1 = (short int)Y[0]&0xFF ;
		unsigned char y2 = (short int)Y[0]>>8 ;
		unsigned char z1 = (short int)Z[0]&0xFF ;
		unsigned char z2 = (short int)Z[0]>>8 ;
                
              

		if(flag==true) {
			USART_SendData(USART1,x1);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			USART_SendData(USART1,x2);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			USART_SendData(USART1,y1);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			USART_SendData(USART1,y2);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			USART_SendData(USART1,z1);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
			
			USART_SendData(USART1,z2);
			while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
		}

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}
#ifdef __cplusplus
}
#endif