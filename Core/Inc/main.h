/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define  ture   1
#define  false  0
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIMIT2_Pin GPIO_PIN_2
#define LIMIT2_GPIO_Port GPIOE
#define LIMIT3_Pin GPIO_PIN_3
#define LIMIT3_GPIO_Port GPIOE
#define LIMIT4_Pin GPIO_PIN_4
#define LIMIT4_GPIO_Port GPIOE
#define LIMIT5_Pin GPIO_PIN_5
#define LIMIT5_GPIO_Port GPIOE
#define LIMIT6_Pin GPIO_PIN_6
#define LIMIT6_GPIO_Port GPIOE
#define LIMIT7_Pin GPIO_PIN_13
#define LIMIT7_GPIO_Port GPIOC
#define DEVICE_AUTO_STOP_Pin GPIO_PIN_14
#define DEVICE_AUTO_STOP_GPIO_Port GPIOC
#define DEVICE_AUTO_STOP_EXTI_IRQn EXTI15_10_IRQn
#define LIMIT9_Pin GPIO_PIN_15
#define LIMIT9_GPIO_Port GPIOC
#define TMC_EN1_Pin GPIO_PIN_2
#define TMC_EN1_GPIO_Port GPIOA
#define SPI1_CSN1_Pin GPIO_PIN_3
#define SPI1_CSN1_GPIO_Port GPIOA
#define TMC_EN2_Pin GPIO_PIN_4
#define TMC_EN2_GPIO_Port GPIOA
#define SPI1_CSN2_Pin GPIO_PIN_4
#define SPI1_CSN2_GPIO_Port GPIOC
#define SPI1_CSN3_Pin GPIO_PIN_5
#define SPI1_CSN3_GPIO_Port GPIOC
#define SPI1_CSN4_Pin GPIO_PIN_0
#define SPI1_CSN4_GPIO_Port GPIOB
#define SPI1_CSN5_Pin GPIO_PIN_1
#define SPI1_CSN5_GPIO_Port GPIOB
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define TMC_EN4_Pin GPIO_PIN_13
#define TMC_EN4_GPIO_Port GPIOB
#define TMC_EN3_Pin GPIO_PIN_14
#define TMC_EN3_GPIO_Port GPIOB
#define TMC_EN5_Pin GPIO_PIN_15
#define TMC_EN5_GPIO_Port GPIOB
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define I2C3_SCL_Pin GPIO_PIN_8
#define I2C3_SCL_GPIO_Port GPIOA
#define CPU_MODE0_Pin GPIO_PIN_15
#define CPU_MODE0_GPIO_Port GPIOA
#define CPU_MODE1_Pin GPIO_PIN_10
#define CPU_MODE1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_11
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_2
#define LED4_GPIO_Port GPIOD
#define LED5_Pin GPIO_PIN_3
#define LED5_GPIO_Port GPIOD
#define BK0_Pin GPIO_PIN_7
#define BK0_GPIO_Port GPIOD
#define RESET_NEW_Pin GPIO_PIN_3
#define RESET_NEW_GPIO_Port GPIOB
#define RESET9539_Pin GPIO_PIN_4
#define RESET9539_GPIO_Port GPIOB
#define KEY_CHANG_Pin GPIO_PIN_5
#define KEY_CHANG_GPIO_Port GPIOB
#define KEY_CHANG_EXTI_IRQn EXTI9_5_IRQn
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define LIMIT0_Pin GPIO_PIN_0
#define LIMIT0_GPIO_Port GPIOE
#define LIMIT1_Pin GPIO_PIN_1
#define LIMIT1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */


#define PCA9539_INT_Pin GPIO_PIN_4

#define PCA9539_INT_GPIO_Port BK2_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
