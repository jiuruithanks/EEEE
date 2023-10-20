/**
 * @file led_key_tasks.c
 * @author Zhao Haofei
 * @brief 灯和按键控制相关任务
 * @version 1.0
 * @date 2023-04-18
 *
 * @copyright Copyright (c) 2023
 */
#include "main.h"
#include "led_key_tasks.h"
#include "tmc5160.h"
#include "bsp.h"

/****************************************************************************
 * 
 * this program is used to change the state of the  open or close 
 * 
 ****************************************************************************/
void Update_Key_status( PCA9555_HandleTypeDef *hdev_key, PCA9555_HandleTypeDef *hdev_led)
{
	//1:代表按键按下
	for (uint8_t i = 0; i < 8; i++)
	{
		if(hdev_key->key[i] == 1 && hdev_key->key_pre[i] == 0)
		{	
			if(hdev_key->key_action[i] == 0)
			{
				hdev_key->key_action[i] = 1;
			}
			else if(hdev_key->key_action[i] == 1)
			{
				hdev_key->key_action[i] = 0;
			}
		}
	}

	
	/* 导丝相关 */
	if ((hdev_key->key_action[2] == 1) && (hdev_key->key_action[1] == 0))
	{
		gw_move.motor_move_head_flag = 1;
		gw_move.motor_move_back_flag = 0;
	}
	else if ((hdev_key->key_action[2] == 0) && (hdev_key->key_action[1] == 1))
	{
		gw_move.motor_move_head_flag = 0;
		gw_move.motor_move_back_flag = 1;
	}
	else if ((hdev_key->key_action[2] == 1) && (hdev_key->key_action[1] == 1))
	{
		gw_move.motor_move_head_flag = 0;
		gw_move.motor_move_back_flag = 0;
		hdev_key->key_action[1] = 0;
		hdev_key->key_action[2] = 0;
	}
	else
	{
		gw_move.motor_move_head_flag = 0;
		gw_move.motor_move_back_flag = 0;
	}
	
	

	/* 器械相关 */
	if((hdev_key->key[4] == 1) && (hdev_key->key_pre[4] == 0))
	{
		// if(devicex->motor_auto_run_flag==0)
		// devicex->motor_auto_run_flag=1;
		device_move.motor_move_head_flag = hdev_key->key_action[4];
		device_move.motor_move_back_flag = 0;
	}
	if((hdev_key->key[3] == 1) && (hdev_key->key_pre[3] == 0))
	{
		// if(devicex->motor_auto_run_flag==0)
		// devicex->motor_auto_run_flag=1;
		device_move.motor_move_head_flag = 0;
		device_move.motor_move_back_flag = hdev_key->key_action[3];
	}
}

/// @brief  低8位按键解码，高位低4bit解码旋扭
///  低8位的解码，来自PCA9555的8位按键   IO低电平时表示触发，进入MCU的信号高电平表示按键按下
///  BIT11~BIT8，来源于旋扭开关，档位参照编码
/// @param hdev 
uint8_t led_read_test;
void pca9555_decode(PCA9555_HandleTypeDef *hdev)
{
	uint8_t rotate_temp;
	uint8_t rotate_out;

	for(uint8_t i = 0; i < 16; i++)
	{
		hdev->key_pre[i] = hdev->key[i];
	}

	for(uint8_t i = 0; ((i == 8 || i == 9 || i == 10 || i == 11) ? i++ : i) < 16; i++)
	{
		hdev->key[i] = ((((hdev->key_out) >> i) & 0x01) == 0) ? 1 : 0;
	}

	
	rotate_temp=(hdev->key_out >> 8) & 0x0F;

	switch (rotate_temp)
	{
		case 0 : rotate_out = 1;		hdev->rotate_bit = 1;		break;
		case 1 : rotate_out = 2;		hdev->rotate_bit = 1 << 1;	break;
		case 3 : rotate_out = 3;		hdev->rotate_bit = 1 << 2;	break;
		case 2 : rotate_out = 4;		hdev->rotate_bit = 1 << 3;	break;
		case 6 : rotate_out = 5;		hdev->rotate_bit = 1 << 4;	break;
		case 7 : rotate_out = 6;		hdev->rotate_bit = 1 << 5;	break;
		case 5 : rotate_out = 7;		hdev->rotate_bit = 1 << 6;	break;
		case 4 : rotate_out = 8;		hdev->rotate_bit = 1 << 7;	break;
		case 0X0C : rotate_out = 9;		hdev->rotate_bit = 1 << 8;	break;
		case 0X0D : rotate_out = 10;	hdev->rotate_bit = 1 << 9;	break;
		case 0X09 : rotate_out = 11;	hdev->rotate_bit = 1 << 10;	break;
		case 0X08 : rotate_out = 12;	hdev->rotate_bit = 1 << 11;	break;
		default:rotate_out = 0;		break;
	}
	hdev->rotate = rotate_out;

	
	
}

///更新LED状态
////****按键取反，按键上升沿时，读取LED状态，取反---8个按键
////****旋转旋扭档位指示 ，档位上亮灯，其它灯为灭档位1~12档，对应LED0~LED10,
///组合时，将第二个循环从8开始，0~7,按键取反，8~10使用档位
void Update_I2CLed( PCA9555_HandleTypeDef *hdev_key, PCA9555_HandleTypeDef *hdev_led)
{
	if (hdev_key->key_equal_flag == 1)
	{
		hdev_key->key_equal_flag = 0;
		
		if(hdev_key->key_action[0] == 1)
		{
			// pca9555_DigitalWrite(hdev_led, 0, GPIO_PIN_SET);
			// pca9555_DigitalWrite(hdev_led, 1, GPIO_PIN_RESET);

			PCA9539_DigitalWrite(hdev_led, 0, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 1, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 0, GPIO_PIN_RESET);
			// pca9555_DigitalWrite(hdev_led, 1, GPIO_PIN_SET);

			PCA9539_DigitalWrite(hdev_led, 0, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 1, GPIO_PIN_SET);
		}
		if(hdev_key->key_action[5] == 1)
		{
			// pca9555_DigitalWrite(hdev_led, 7, GPIO_PIN_SET);
			// pca9555_DigitalWrite(hdev_led, 8, GPIO_PIN_RESET);

			PCA9539_DigitalWrite(hdev_led, 7, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 8, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 7, GPIO_PIN_RESET);
			// pca9555_DigitalWrite(hdev_led, 8, GPIO_PIN_SET);

			PCA9539_DigitalWrite(hdev_led, 7, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 8, GPIO_PIN_SET);
		}
		if((hdev_key->key_action[6] == 1) && (hdev_key->rotate == 5))
		{
			PCA9539_DigitalWrite(hdev_led, 9, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 10, GPIO_PIN_RESET);
		}
		else 
		{
			PCA9539_DigitalWrite(hdev_led, 9, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 10, GPIO_PIN_SET);
		}
	
		if (hdev_key->rotate == 8)
		{
			// pca9555_DigitalWrite(hdev_led, 2, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 2, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 2, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 2, GPIO_PIN_SET);
		}

		if (hdev_key->rotate == 7)
		{
			// pca9555_DigitalWrite(hdev_led, 3, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 3, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 3, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 3, GPIO_PIN_SET);
		}
		
		if (hdev_key->rotate == 6)
		{
			// pca9555_DigitalWrite(hdev_led, 4, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 4, GPIO_PIN_RESET);
		}
		else
		{
			// pca9555_DigitalWrite(hdev_led, 4, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 4, GPIO_PIN_SET);

		}
		
		if (hdev_key->rotate == 5)
		{
			// pca9555_DigitalWrite(hdev_led, 5, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 5, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 5, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 5, GPIO_PIN_SET);
		}
		
		if (hdev_key->rotate == 4)
		{
			// pca9555_DigitalWrite(hdev_led, 6, GPIO_PIN_RESET);
			PCA9539_DigitalWrite(hdev_led, 6, GPIO_PIN_RESET);
		}
		else 
		{
			// pca9555_DigitalWrite(hdev_led, 6, GPIO_PIN_SET);
			PCA9539_DigitalWrite(hdev_led, 6, GPIO_PIN_SET);
		}
	}		
}


void I2C_LEDON(PCA9555_HandleTypeDef *hdev, uint8_t num)
{
	// pca9555_DigitalWrite(hdev, num, GPIO_PIN_RESET);
	PCA9539_DigitalWrite(hdev, num, GPIO_PIN_RESET);
}
void I2C_LEDOFF(PCA9555_HandleTypeDef *hdev,uint8_t num)
{
	// pca9555_DigitalWrite(hdev, num, GPIO_PIN_SET);
	PCA9539_DigitalWrite(hdev, num, GPIO_PIN_SET);
}


void Changeled(PCA9555_HandleTypeDef *hdev, uint8_t num)
{
	uint8_t bit_value;
	 
	// pca9555_digitalRead(hdev, num, &bit_value);
	PCA9539_DigitalRead(hdev, num, &bit_value);
	if(bit_value == 0)
	{
		// pca9555_DigitalWrite(hdev, num, GPIO_PIN_SET);
		PCA9539_DigitalWrite(hdev, num, GPIO_PIN_SET);
	}
	else if(bit_value==1)
	{		
		// pca9555_DigitalWrite(hdev, num, GPIO_PIN_RESET);
		PCA9539_DigitalWrite(hdev, num, GPIO_PIN_RESET);
	}
	if (num < 8)
	{
		Step_Motor_Flag_pre[num] = Step_Motor_Flag[num];
		Step_Motor_Flag[num] = bit_value;
	}
}


