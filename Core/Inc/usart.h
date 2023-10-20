/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define MAX_REC_LENGTH  64 
#define LENGTH  32 	
	
  
extern unsigned char UART1_Rx_Buf[MAX_REC_LENGTH];
extern unsigned char UART1_Rx_flg;
extern unsigned int  UART1_Rx_cnt;
extern unsigned char UART1_Rx_Buf_reg[MAX_REC_LENGTH]; //USART1�洢��������

extern uint8_t  UART1_Tx_Buf[LENGTH];
extern uint8_t	UART1_Tx_Buf_reg[LENGTH];

extern uint32_t UART3_Rx_Buf[MAX_REC_LENGTH];
extern uint8_t  UART3_Rx_flg;
extern uint32_t UART3_Rx_cnt;
extern uint32_t UART3_Rx_Buf_reg[MAX_REC_LENGTH]; //USART3�洢��������

extern  unsigned char delete_esc_data[64];  //ɾ��ת���ַ�
extern	uint8_t g_insert_esc_data[64];      //����ת���ַ�
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

