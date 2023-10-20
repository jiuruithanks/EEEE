/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp.h"
#include "tmc5160.h"
#include "bsp_key.h"
#include "pca9539.h"
#include "bsp_as5048b.h"
#include "bsp_led.h"
#include "bsp_dcmotor.h"
#include "bsp_pid.h"
#include "task.h"
#include "led_key_tasks.h"
#include "motor_tasks.h"
#include "myi2c.h"
#include "24cxx.h"
#include "XGZP6877D.h"
#include "status_monitor_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t mcu_mode;
uint16_t sys_cnt_uart_motor;
uint16_t target;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	at24cxx_init();
	PID_init();		
//	xgzp6877d_init(&xgzp6877d_handle);
	XGZP6877D_Init();
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断	
	HAL_UART_Receive_DMA(&huart1,UART1_Rx_Buf,MAX_REC_LENGTH);	
	
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能IDLE中断	
	HAL_UART_Receive_DMA(&huart3,(uint8_t *)UART3_Rx_Buf,MAX_REC_LENGTH);
 
 ///get cpu mode///
	mcu_mode = Get_Mode();
//	HAL_GPIO_WritePin(GPIOB, RESET9539_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, RESET_NEW_Pin,GPIO_PIN_SET);
 
 //*STEP MOTOR */


	Tmc5160Initial(&SPICS1);
	Tmc5160Initial(&SPICS2);
	Tmc5160Initial(&SPICS3);
	Tmc5160Initial(&SPICS4);
//	Tmc5160Initial(&SPICS5);

	Dis_All_Step_Driver();

/*DC motor*/

//	bsp_InitKey();

  /*uart motor */

	uart_step_inital();

/*I2C KEY AND LED*/

//	pca9555_init(&PCA9555_LED, &hi2c3, 0x74);
//	pca9555_init(&PCA9555_KEY, &hi2c3, 0x75);
	PCA9539_init();

  /*AS5048B*/
  as5048b_init(&as5048b00);
  as5048b_init(&as5048b01);
  as5048b_init(&as5048b10);
  as5048b_init(&as5048b11);

//	bsp_AS5048B_Init(&as5048b00, 0x40, &hi2c1);
//	bsp_AS5048B_Init(&as5048b01, 0x41, &hi2c1);
//	bsp_AS5048B_Init(&as5048b10, 0x42, &hi2c1);
//	bsp_AS5048B_Init(&as5048b11, 0x43, &hi2c1);

//	bsp_hsppadx4_Init(&hsppad147_handle, &hsppad147_config);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim9);
	HAL_TIM_Base_Start_IT(&htim10);

	PCA9539_DigitalRead(&PCA9555_KEY, 0, &bit_value);
	Update_Key_status(&PCA9555_KEY, &PCA9555_LED);
	pca9555_decode(&PCA9555_KEY);
	Update_I2CLed(&PCA9555_KEY, &PCA9555_LED);		
	
  mode_select(INJECT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
    
	while (1)
	{
    /* 注射库控制时要屏蔽PCA9555_KEY相关 */
  #if INJECT
//		key_step_inject(); 
//		key_dc_inject();
		injection_control(&encoder_select);
//		xgzp6877d_init(&xgzp6877d_handle);
//		
		Pressure_Temperature_Cal(&xgzp6877d_handle);
  #else	
		Update_Motor_Status(&PCA9555_KEY, &gw_move,&device_move, &device_action, &device_and_gw_move);
		Change_Device(&SPICS3, &PCA9555_KEY);

  #endif
             
		
		if (1 == bsp_RunPer10ms_flag)
		{ 
		  bsp_RunPer10ms_flag = 0;
		  bsp_RunPer10ms();
		}
 
    if (bsp_RunPer50ms_flag == 1)
    {
      bsp_RunPer50ms_flag = 0;
		  bsp_RunPer50ms();
    }
    
		if (1 == bsp_RunPer100ms_flag)
		{
		  bsp_RunPer100ms_flag = 0;
		  bsp_RunPer100ms();
		}
		
    if (bsp_RunPer200ms_flag == 1)
		{
		  bsp_RunPer200ms_flag = 0;
		  bsp_RunPer200ms();
		}

		if (1 == bsp_RunPer500ms_flag)
		{
		  bsp_RunPer500ms_flag = 0;
		  bsp_RunPer500ms();
		}

		// if(UART1_Rx_flg==1)
		// {
		//   Check_RecData(UART1_Rx_Buf_reg,UART1_Rx_cnt);
		// }
	
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
