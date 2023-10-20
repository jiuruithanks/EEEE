#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "main.h"

#define LED0_OFF     HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET)
#define LED0_ON      HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET)
#define LED1_OFF     HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED1_ON      HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED2_OFF     HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)
#define LED2_ON      HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET)
	
#define LED3_OFF     HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)
#define LED3_ON      HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)
#define LED4_OFF     HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)
#define LED4_ON      HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)
#define LED5_OFF     HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_SET)
#define LED5_ON      HAL_GPIO_WritePin(LED5_GPIO_Port,LED5_Pin,GPIO_PIN_RESET)

void Switch_LED(uint8_t num);

#endif



