#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
  

int main(void)

   {
    //Clock Configuration
     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    //LED Configuration
     STM_EVAL_LEDInit(LED6);
   //Push-Button Configuration
    STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);

     //Main Loop
    while(true) 
      {
      //Get push-button state
      unsigned int n = STM_EVAL_PBGetState(BUTTON_USER);
      if(n==0) STM_EVAL_LEDOff(LED6);//enable blue LED
        else 
            STM_EVAL_LEDOn(LED6); //disable blue LED
    }

   }

