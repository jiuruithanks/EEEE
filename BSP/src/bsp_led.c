#include "bsp_led.h"
#include "main.h"



void Switch_LED(uint8_t num)
{
  uint8_t led_num;
  led_num=num;
  switch (led_num)
  {
  case 0: LED0_OFF;LED1_OFF;LED2_OFF  ;LED3_OFF;LED4_OFF;LED5_OFF	;break;
  case 1: LED0_OFF;LED1_OFF;LED2_ON	 	;LED3_OFF;LED4_OFF;LED5_ON	 ;break;
  case 2: LED0_OFF;LED1_ON;LED2_OFF		;LED3_OFF;LED4_ON ;LED5_OFF ;break;
  case 3: LED0_OFF;LED1_ON;LED2_ON		;LED3_OFF;LED4_ON ;LED5_ON ;break;
  case 4: LED0_ON;LED1_OFF;LED2_OFF		;LED3_ON;LED4_OFF;LED5_OFF ;break;
  case 5: LED0_ON;LED1_OFF;LED2_ON		;LED3_ON;LED4_OFF;LED5_ON;break;
  case 6: LED0_ON;LED1_ON;LED2_OFF		;LED3_ON;LED4_ON;LED5_OFF;break;
  case 7: LED0_ON;LED1_ON;LED2_ON			;LED3_ON;LED4_ON;LED5_ON; break;
  default:		   break;
  }
}

