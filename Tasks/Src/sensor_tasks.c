/**
 * @file sensor_tasks.c
 * @author Zhao Haofei
 * @brief �������������
 * @version 1.0
 * @date 2023-08-22
 *
 * @copyright Copyright (c) 2023
 */
#include "sensor_tasks.h"
#include "math.h"



/**
 * @brief ��˿����е�����ɣ���ز�����ʼ��
 * 
 * @param DCmotor 
 * @param as5048bxx 
 */
void high_transition_low(DCmotor_TypeDef* DCmotor, AS5048B_HandleTypeDef* as5048bxx)
{  
	as5048bxx->arrive_pos = HAL_GPIO_ReadPin(DCmotor->head_limit_GPIOx,DCmotor->head_limit_GPIO_Pin); 

	if (as5048bxx->arrive_pos_pre == 1)
	{
		if (as5048bxx->arrive_pos == 0)
		{
			as5048bxx->pos_head_move_after_zero_process = 0;
			as5048bxx->angle_one = 0;
			as5048bxx->Cycle = 0;
		}
	}

	as5048bxx->arrive_pos_pre = as5048bxx->arrive_pos;
}


/**
 * @brief ���㵼˿����е�Ľ��˳���
 * 
 * @param as5048bxx 
 */
void cal_wire_device_length(AS5048B_HandleTypeDef *as5048bxx)
{
	//ͳһ��������������
	if (as5048bxx->ADDRESS == ADDR_A)
	{
		as5048bxx->pos_head_move_ori = as5048bxx->angle_h;
	}
	else if (as5048bxx->ADDRESS == ADDR_B)
	{
		as5048bxx->pos_head_move_ori = 0x3FFF - as5048bxx->angle_h;
	}

	if (as5048bxx->first_flag == 0)
	{
		as5048bxx->pos_head_move_ori_pre = as5048bxx->pos_head_move_ori;
		as5048bxx->first_flag = 1;
		return;
	}
	
	
	//����㴦��
	if (((as5048bxx->pos_head_move_ori & 0x3000) == 0x3000) && ((as5048bxx->pos_head_move_ori_pre & 0x3000) == 0))	//back
	{																					//(& 0xffffc000) ��������������ʱ��12�� 9 + 12 = 21 ȡ�� 12��
		as5048bxx->pos_head_move_after_zero_process = as5048bxx->pos_head_move_after_zero_process - (0x4000 - as5048bxx->pos_head_move_ori + as5048bxx->pos_head_move_ori_pre); //������ + pos_head_move_ori
	}
	else if (((as5048bxx->pos_head_move_ori & 0x3000) == 0) && ((as5048bxx->pos_head_move_ori_pre & 0x3000) == 0x3000)) //head
	{
		as5048bxx->pos_head_move_after_zero_process = as5048bxx->pos_head_move_after_zero_process + (as5048bxx->pos_head_move_ori + 0x4000 - as5048bxx->pos_head_move_ori_pre); //������ + pos_head_move_ori
	} 
	else 
	{	//�������Ĵ���
		as5048bxx->pos_head_move_after_zero_process = as5048bxx->pos_head_move_after_zero_process + as5048bxx->pos_head_move_ori - as5048bxx->pos_head_move_ori_pre; //������ + pos_head_move_ori
	}

	as5048bxx->pos_head_move_ori_pre = as5048bxx->pos_head_move_ori;
	//ת����Ȧ��
	as5048bxx->Cycle = (as5048bxx->pos_head_move_after_zero_process / 0x4000);
	//����һȦ�ĽǶ�
	as5048bxx->angle_one = as5048bxx->pos_head_move_after_zero_process % 0x4000;
	//����һȦ�Ľ��˳���
	as5048bxx->length = (as5048bxx->angle_one * 360.0 / 16383) * 0.01745 * R;
	//�ܵĽ��˳���
	as5048bxx->sum_length = (as5048bxx->Cycle * 2 * Pi * R) + as5048bxx->length;

}


/**
 * @brief ��˿����е��ʵ��ת��
 * 
 * @param as5048bxx 
 */
void motor_real_speed(AS5048B_HandleTypeDef* as5048bxx)
{
	
	as5048bxx->angle_rpm_pre = as5048bxx->angle_rpm;
	AS5048B_Read(as5048bxx, as5048bxx->ADDRESS, &angle_as5048b_h);
    as5048bxx->angle_rpm = as5048bxx->angle_f;
	

	as5048bxx->real_speed = fabs(as5048bxx->angle_rpm - as5048bxx->angle_rpm_pre) / 360 * 20 * 60;
}


