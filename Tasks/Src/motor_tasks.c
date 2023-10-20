/**
 * @file step_motor_tasks.c
 * @author Zhao Haofei
 * @brief ������������������
 * @version 1.0
 * @date 2022-04-18
 *
 * @copyright Copyright (c) 2023
 */
#include <math.h>
#include "motor_tasks.h"
#include "bsp_pid.h"
#include "encoder.h"
/*********************************************** ȫ�ֱ��� **********************************************/
uint8_t channel_temp; //������е����ť��λ

uint8_t device1_dir; 	//�и����
uint8_t device1_dir_pre;
float device1_tar_pos; 		// target position
float device1_tar_pos_pre;
uint16_t g_travel = Uartmotor3_maxtarget;	//�иʵ��λ��
float g_increment;	//�г�����


KEY_VAL key_val; //����ֵ
uint8_t g_vol_val; //��ѹֵ


TickTimer bileDuctTimer = {0, 0};


/*********************************************** ERCP��е������� **********************************************/
/**
 * @brief ���ڱ�����е�л�λ��
 * 
 * @param CS 
 * @param key 
 */
void Change_Device(SPI_CS_TypeDef *CS, PCA9555_HandleTypeDef *key)
{
	SPI_CS_TypeDef *p = CS;
	PCA9555_HandleTypeDef *key_temp = key;

	switch (key_temp->rotate)
	{
		case  8 : channel_temp = 0; break;
		case  7 : channel_temp = 1; break;
		case  6 : channel_temp = 2; break;
		case  5 : channel_temp = 3; break;
		case  4 : channel_temp = 3; break;
	default : channel_temp = channel_temp; break;
	}

	if (key_temp->rotate == 4)
	{
		HAL_GPIO_WritePin(p->EN_GPIOx, p->EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		Run_STEP_Motor_POS(p, channel_temp);
	}	
}


/// @brief ����ERCP��е��ǰ����λ
/// @param hdev_key   		����������Ť���ƣ��л�1��2��3��4����е
/// @param hdev_led   		�������LED
/// @param device_actionx   ����1�š�3����е���� 
void Updata_Current_device(PCA9555_HandleTypeDef *hdev_key, PCA9555_HandleTypeDef *hdev_led, DEVICE_GW_Typedef *device_actionx)
{
	uint8_t current_device = 0;
	uint8_t device4_dir = 0;
	uint8_t device4_rec_angle = 0;
	uint8_t device5_dir = 0;
	uint8_t device5_rec_angle = 0;

	switch (hdev_key->rotate)
	{
		case  8 :current_device = 1;		break;
		case  7 :current_device = 2;		break;
		case  6 :current_device = 3;		break;
		case  5 :current_device = 4;		break;
		case  4 :current_device = 5;		break;
		default :current_device = 0;		break;
	}

	if(current_device == 1)
	{
		if(device_actionx->run_uartflag == 1)
		{
			
			device1_dir = device_actionx->run_dir;                //���ڵ����������
			device1_tar_pos = device_actionx->rec_data;				//���յ���е�����Ŀ��ƽǶ���Ϣ	target position    angle of knob
			
			if((device1_dir == 0) && (device1_tar_pos < LIMIT_LEFT))	//anticlockwise ��ʱ��
			{			
				// g_travel = (Uartmotor3_maxtarget - Uartmotor3_mintarget) * device1_tar_pos / 97 + Uartmotor3_mintarget;		//Absolute form				
				g_travel -= g_increment;																						//incremental
				if (g_travel < Uartmotor3_mintarget)
				{
					g_travel = Uartmotor3_mintarget;
				}				
				else if(g_travel > Uartmotor3_maxtarget)
				{
					g_travel = Uartmotor3_maxtarget;
				}
				Update_uartmotor_target(g_travel);
				HAL_UART_Transmit(&huart3, Uartmotor3_target, sizeof(Uartmotor3_target), 1);													
			}
			else if ((device1_dir == 1) && (device1_tar_pos > LIMIT_RIGHT))	//clockwise	˳ʱ��
			{
				// g_travel = (Uartmotor3_maxtarget - Uartmotor3_mintarget) * device1_tar_pos / 255 + Uartmotor3_mintarget;		//Absolute form				
				g_travel += g_increment;			//incremental
				if (g_travel < Uartmotor3_mintarget)
				{
					g_travel = Uartmotor3_mintarget;
				}
				else if(g_travel > Uartmotor3_maxtarget)
				{
					g_travel = Uartmotor3_maxtarget;
				}
				Update_uartmotor_target(g_travel);
				HAL_UART_Transmit(&huart3, Uartmotor3_target, sizeof(Uartmotor3_target), 1);															
			}
			device1_tar_pos_pre = device1_tar_pos;
		}
	}

	
	if(current_device == 2)
	{
		
	}

	if(current_device == 3)
	{

		if((device_actionx->run_dir == 0) && (device_actionx->run_uartflag == 1)) 
		{
			SPICS4.vmax = (SPICS4.vmax_default / 100) * device_actionx->run_speed; 
			Run_STEP_Motor_SPEED(&SPICS4, back);
		}
		else if ((device_actionx->run_dir == 1) && (device_actionx->run_uartflag == 1))
		{
			SPICS4.vmax = (SPICS4.vmax_default / 100) * device_actionx->run_speed; 
			Run_STEP_Motor_SPEED(&SPICS4, head);
		}
		else
		{
			Stop_STEP_Motor(&SPICS4);	
		}
		check_motor_status(&SPICS4);
	}
	else
	{
		Stop_STEP_Motor(&SPICS4);
	}

	if(current_device == 4)
	{
		device4_dir = device_actionx->run_dir;
		device4_rec_angle = device_actionx->rec_data;

		if ((device4_dir == 0) && (device4_rec_angle < LIMIT_LEFT))
		{
			Run_DC_Motor(&DCmotor3, back, 12);	
		}
		else if ((device4_dir == 1) && (device4_rec_angle > LIMIT_RIGHT))
		{
			Run_DC_Motor(&DCmotor3, head, 12);
		} 
	}
	else
	{
	 Run_DC_Motor(&DCmotor3, head, 12);
	}	

	if (current_device == 5)
	{
		AS5048B_Read(&as5048b11, 0xFE, &angle_as5048b_h);

		device5_dir = device_actionx->run_dir;
		device5_rec_angle = device_actionx->rec_data;

		if((device5_dir == 0) && (device5_rec_angle < LIMIT_LEFT)) 
		{
			SPICS5.vmax = (SPICS5.vmax_default / 100) * device_actionx->run_speed; 
			Run_STEP_Motor_SPEED(&SPICS5, back);
		}
		else if ((device5_dir == 1) && (device5_rec_angle > LIMIT_RIGHT))
		{
			SPICS5.vmax = (SPICS5.vmax_default / 100) * device_actionx->run_speed; 
			Run_STEP_Motor_SPEED(&SPICS5, head);
		}
		else
		{
			Stop_STEP_Motor(&SPICS5);	
		}
	//	check_motor_status(&SPICS5);
	}
	else
	{
		Stop_STEP_Motor(&SPICS5);
	}
	
	
}



/// @brief	control gw and device move
/// @param hdev_key    key to control the device or gw  move head or back
/// @param gwx         uart message to contorl  gw move head or back 
/// @param devicex     uart message to contol device  head or back
/// @param device_and_gw_actionx	gw and device combined action
void Update_step_motor(PCA9555_HandleTypeDef *hdev_key, DEVICE_GW_Typedef *gwx, DEVICE_GW_Typedef *devicex, DEVICE_GW_Typedef *device_and_gw_actionx)
{
	/* ��˿������� */
	if((( hdev_key->key[7] == 1) || ((gwx->run_dir == 1) && (gwx->run_uartflag == 1))) || ((device_and_gw_actionx->gw_dev_flag == 0) && (device_and_gw_actionx->dev_gw_flag == 1)))
	{
		if(hdev_key->key[7] == 1)
		{
			SPICS1.amax = 100;
			SPICS1.vmax = SPICS1.vmax_default;
		}
		else if ((device_and_gw_actionx->gw_dev_flag == 0) && (device_and_gw_actionx->dev_gw_flag == 1))
		{
			SPICS1.amax = 1000;
			SPICS1.vmax = (SPICS1.vmax_default / 100) * (device_and_gw_actionx->run_speed); 
		}
		else
		{
			SPICS1.amax = 1000;
			SPICS1.vmax = (SPICS1.vmax_default / 100) * (gwx->run_speed); 
		}
		Run_STEP_Motor_SPEED(&SPICS1 ,1);	//��˿��
	}
	else if(((hdev_key->key[12] == 1) || ((gwx->run_dir == 0) && (gwx->run_uartflag == 1))) || ((device_and_gw_actionx->gw_dev_flag == 1) && (device_and_gw_actionx->dev_gw_flag == 0)))
	{
		if(hdev_key->key[12] == 1)
		{
			SPICS1.amax = 100;
			SPICS1.vmax = SPICS1.vmax_default;
		}
		else if ((device_and_gw_actionx->gw_dev_flag == 1) && (device_and_gw_actionx->dev_gw_flag == 0))
		{
			SPICS1.amax = 1000;
			SPICS1.vmax = (SPICS1.vmax_default / 100) * (device_and_gw_actionx->run_speed); 
		}
		else
		{
			SPICS1.amax = 1000;
			SPICS1.vmax = (SPICS1.vmax_default / 100) * (gwx->run_speed); 
		}
		Run_STEP_Motor_SPEED(&SPICS1 ,0);	//��˿��
	}
	else if ((gwx->motor_move_head_flag == 1) && (gwx->motor_move_back_flag == 0))
	{
		SPICS1.amax = 50;
		SPICS1.vmax = SPICS1.vmax_default;
		Run_STEP_Motor_SPEED(&SPICS1, 0);
	}
	else if((gwx->motor_move_head_flag == 0) && (gwx->motor_move_back_flag == 1))
	{
		SPICS1.amax = 50;
		SPICS1.vmax = SPICS1.vmax_default;
		Run_STEP_Motor_SPEED(&SPICS1, 1);
	}
	else
	{
		Stop_STEP_Motor(&SPICS1);
	}

	/* ��е������� */
	if (((hdev_key->key[12] == 1) || ((devicex->run_dir == 1) && (devicex->run_uartflag == 1))) || ((device_and_gw_actionx->gw_dev_flag == 1) && (device_and_gw_actionx->dev_gw_flag == 0)))
	{
		if (hdev_key->key[12] == 1)
		{
			SPICS2.amax = 50;
			SPICS2.vmax = SPICS2.vmax_default;
		}
		else if ((device_and_gw_actionx->gw_dev_flag == 1) && (device_and_gw_actionx->dev_gw_flag == 0))
		{
			SPICS2.amax = 1000;
			SPICS2.vmax = (SPICS2.vmax_default / 100) * device_and_gw_actionx->run_speed;
		}
		else
		{
			SPICS2.amax = 1000;
			SPICS2.vmax = (SPICS2.vmax_default / 100) * devicex->run_speed;    
		}
		Run_STEP_Motor_SPEED(&SPICS2 ,1);	//��е��
	}
	else if ((devicex->motor_move_head_flag == 1)&&(devicex->motor_move_back_flag == 0))
	{
		SPICS2.amax = 50;
		SPICS2.vmax = SPICS2.vmax_default;
		Run_STEP_Motor_SPEED(&SPICS2 ,0);
	}
	else if((devicex->motor_move_head_flag == 0)&&(devicex->motor_move_back_flag == 1))
	{
		SPICS2.amax = 50;
		SPICS2.vmax = SPICS2.vmax_default;
		Run_STEP_Motor_SPEED(&SPICS2 ,1);
	}
	else if(((hdev_key->key[7] == 1) || ((devicex->run_dir == 0) && (devicex->run_uartflag == 1))) || ((device_and_gw_actionx->gw_dev_flag == 0) && (device_and_gw_actionx->dev_gw_flag == 1)))
	{
		if(hdev_key->key[7] == 1)
		{
			SPICS2.amax = 50;
			SPICS2.vmax = SPICS2.vmax_default;
		}
		else if ((device_and_gw_actionx->gw_dev_flag == 0) && (device_and_gw_actionx->dev_gw_flag == 1))
		{
			SPICS2.amax = 1000;
			SPICS2.vmax = (SPICS2.vmax_default / 100) * device_and_gw_actionx->run_speed; 
		}
		else
		{
			SPICS2.amax = 1000;
			SPICS2.vmax = (SPICS2.vmax_default / 100) * devicex->run_speed;   
		}
		Run_STEP_Motor_SPEED(&SPICS2 ,0);	//��е��
	}
	else 
	{
		Stop_STEP_Motor(&SPICS2);	
	}
}






void Update_Motor_Status(PCA9555_HandleTypeDef *hdev_key, DEVICE_GW_Typedef *gwx, DEVICE_GW_Typedef *devicex, DEVICE_GW_Typedef *device_actionx, DEVICE_GW_Typedef *device_and_gw_movex)
{
	Update_step_motor(hdev_key, gwx, devicex, device_and_gw_movex);
	Updata_dcmtor(hdev_key);
	Updata_Current_device(hdev_key, &PCA9555_LED, device_actionx);
}



/*********************************************** ���ڵ���˶�������� **********************************************/

/// @brief  ����Uartmotor3_target ���飬�������ƴ��ڵ��λ��
/// @param tar_pos 
void Update_uartmotor_target(uint16_t tar_pos)
{
	uint16_t temp = 0;
	temp = tar_pos;
	Uartmotor3_target[6] = temp % 256;
	Uartmotor3_target[7] = temp / 256;
	Uartmotor3_target[8] = 0X5D + temp % 256 + temp / 256;
	
	HAL_UART_Transmit(&huart3, Uartmotor3_target, sizeof(Uartmotor3_target), 10);
 	{
 		while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET);		//�ȴ����ͽ���
 	}
}



/************************************************************************** ע����˶�������� *****************************************************************************/
/**
 * @brief ��������ע�����ز���
 * 
 */
void key_step_inject(void)
{
	//���Ƶ���Ķ���
	key_val.keyVal[0] = HAL_GPIO_ReadPin(LIMIT0_GPIO_Port, LIMIT0_Pin);
	key_val.keyVal[1] = HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin);
	key_val.keyVal[4] = HAL_GPIO_ReadPin(LIMIT4_GPIO_Port, LIMIT4_Pin);

	if (key_val.keyVal[0] == 0) 
	{
		Run_STEP_Motor_SPEED(&SPICS1, 1);
	}	
	else 
	{
		Stop_STEP_Motor(&SPICS1);
	}
	if (key_val.keyVal[1] == 0)
	{
		Run_STEP_Motor_SPEED(&SPICS3, 0);
	}			
	else 
	{
		Stop_STEP_Motor(&SPICS3);
	}
	if (key_val.keyVal[4] == 0) 
	{
		Run_STEP_Motor_SPEED(&SPICS2, 0);
	}
	else  
	{
		Stop_STEP_Motor(&SPICS2);
	}
}


/**
 * @brief ��������ע������
 * 
 */
void key_dc_inject(void)
{
  key_val.keyVal[2] = HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin);
  key_val.keyVal[3] = HAL_GPIO_ReadPin(LIMIT3_GPIO_Port, LIMIT3_Pin);
            
  if (key_val.keyVal[2] == 0)
  {
    Run_DC_Motor(&DCmotor1, head, 12);
  }
  else if (key_val.keyVal[3] == 0)
  Run_DC_Motor(&DCmotor1, back, 12);
  else 
  Stop_DC_Motor(&DCmotor1);
}


/**
 * @brief ������ע�����ؿ���
 * 
 * @param inject_action 
 * @param CS 
 */
void panel_step_inject(DEVICE_GW_Typedef *inject_action, SPI_CS_TypeDef *CS)
{
	if (CS == &SPICS1)
	{
		if (inject_action->run_dir == 0)
		{
			CS->vmax = (CS->vmax_default / 100) * (inject_action->run_speed);
			Run_STEP_Motor_SPEED(CS, 1);
		}
		else if (inject_action->run_dir == 1)
		{
			CS->vmax = (CS->vmax_default / 100) * (inject_action->run_speed);
			Run_STEP_Motor_SPEED(CS, 1);
		}
		else
		{
			Stop_STEP_Motor(CS); 
		}
	}
	else
	{
		if (inject_action->run_dir == 0)
		{	
			CS->vmax = (CS->vmax_default / 100) * (inject_action->run_speed);
			Run_STEP_Motor_SPEED(CS, 0);
		}
		else if (inject_action->run_dir == 1)
		{
			CS->vmax = (CS->vmax_default / 100) * (inject_action->run_speed);
			Run_STEP_Motor_SPEED(CS, 0);
		}
		else
		{
			Stop_STEP_Motor(CS); 
		}
	}
}

/**
 * @brief ������ע������
 * 
 * @param inject_action 
 */
void panel_dc_inject(DEVICE_GW_Typedef *inject_action)
{
	float remote_angle = inject_action->rec_data;
	float vol_val = 0;
	if (remote_angle < RECEIVE_DATA_GATE_L)
	{
		vol_val = fabs(127 - remote_angle) / 127 * g_voltage_val;	//ֱ�����������ѹ12V
		Run_DC_Motor(&DCmotor1, back, vol_val);
	}
	else if (remote_angle > RECEIVE_DATA_GATE_H)
	{
		vol_val = fabs(remote_angle - 128) / 127 * g_voltage_val;
		Run_DC_Motor(&DCmotor1, head, vol_val);
	}
	else
	{
		Stop_DC_Motor(&DCmotor1);
	}
}

void injection_control(DEVICE_GW_Typedef *encoder_select)
{
	uint8_t temp = encoder_select->rec_data;
	switch (temp)
	{
	case 1:
		panel_step_inject(&inject_action, &SPICS1);
		Stop_DC_Motor(&DCmotor1);
   		Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	case 2:
		panel_step_inject(&inject_action, &SPICS3);
		Stop_DC_Motor(&DCmotor1);
   		Stop_STEP_Motor(&SPICS1);
		Stop_STEP_Motor(&SPICS2);
		break;
	case 3:
		panel_step_inject(&inject_action, &SPICS2);
		Stop_DC_Motor(&DCmotor1);
   		Stop_STEP_Motor(&SPICS1);
		Stop_STEP_Motor(&SPICS3);
		break;
	case 4:
		panel_dc_inject(&inject_action);
		Stop_STEP_Motor(&SPICS1);
   		Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	default:
		Stop_DC_Motor(&DCmotor1);
		Stop_STEP_Motor(&SPICS1);
   		Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	}
}


void step_test(SPI_CS_TypeDef *CS)
{
	Run_STEP_Motor_SPEED(&SPICS1, 1);
	HAL_Delay(2000);
	Run_STEP_Motor_SPEED(&SPICS1, 0);
	HAL_Delay(2000);
}

/*********************************************** ֱ������˶�������� **********************************************/

/// @brief :ֱ�����1����˿��Ͽ���    ֱ�����2 ��е���״̬ ��λ
/// @param hdev_key ����0�л� ���״̬ ֱ�����1 ��ֱ�����1����λ
void Updata_dcmtor(PCA9555_HandleTypeDef *hdev_key)
{
	if(hdev_key->key_action[0] == 1)
	Run_DC_Motor(&DCmotor1, 0, 12);
	else if (hdev_key->key_action[0] == 0)
	Run_DC_Motor(&DCmotor1, 1, 12);


	if(hdev_key->key_action[5] == 1)
	Run_DC_Motor(&DCmotor2, 0, 12);
	else if (hdev_key->key_action[5] == 0)
	Run_DC_Motor(&DCmotor2, 1, 12);
}



