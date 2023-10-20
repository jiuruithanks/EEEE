/**
 * @file XGZP6877D.c
 * @author Zhao Haofei
 * @brief ѹ������������
 * @version 1.0
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 */
#include "XGZP6877D.h"


XGZP6877D_HANDLE xgzp6877d_handle;


/*********************************************************XGZP6877D����***********************************************************************/ 

//----write One Byte of Data,Data from MASTER to the SLAVER----
void Write_One_Byte(uint8_t addr, uint8_t data) //Write "data" to the SLAVER's address of "addr"
{
//	uint8_t acktemp = 1;
	iic_start(); 					//IIC START
	iic_send_byte((0x6D << 1) + 0); //IIC WRITE operation, SLAVER address bit: 0x6D
	iic_wait_ack(); 				//check the SLAVER
	iic_send_byte(addr); 			//address
	iic_wait_ack(); 			
	iic_send_byte(data); 			//thedata
	iic_wait_ack(); 			
	iic_stop(); 					//IIC STOP
}

//----Read One Byte of Data,Data from SLAVER to the MASTER----
uint8_t Read_One_Byte(uint8_t addr)
{
	uint8_t mydata = 0;
	iic_start();
	iic_send_byte((0x6D << 1) + 0); //IIC WRITE operation, SLAVER address bit: 0x6D
	iic_wait_ack();
	iic_send_byte(addr);
	iic_wait_ack();
	iic_start();
	iic_send_byte((0x6D << 1) + 1); //IIC READ operation, SLAVER address bit: 0x6D
	iic_wait_ack();
	mydata = iic_read_byte(0);
	iic_stop();
	return mydata;
}

void XGZP6877D_Init(void)
{
	iic_init();
}

void Pressure_Temperature_Cal(XGZP6877D_HANDLE *pressure_handle)
{
	/*0x30 ��д�������� 000�� �����¶Ȳ����� 001�� ����ѹ�������� 010�� ��ϣ� ����ѹ�����¶�
������ 011�� ���߷�ʽ����һ����ʱ����ִ�����ģʽ������*/
	Write_One_Byte(0x30, 0x0A);
	//Judge whether Data collection is over �ж����ݲɼ��Ƿ����
	while ((Read_One_Byte(0x30) & 0x08) > 0);
	HAL_Delay(20);
	// Read ADC output Data of Pressure ��ȡ����ѹ��ֵ�� 3 ���Ĵ�����ֵ
	pressure_handle->pressure_H = Read_One_Byte(0x06);
	pressure_handle->pressure_M = Read_One_Byte(0x07);
	pressure_handle->pressure_L = Read_One_Byte(0x08);
	//Compute the value of pressure converted by ADC ���㴫���� ADC ת�����ѹ��ֵ
	pressure_handle->pressure_adc = pressure_handle->pressure_H * 65536 + pressure_handle->pressure_M * 256 + pressure_handle->pressure_L;
	//The conversion formula of calibrated pressure�� its unit is Pa ��������У׼���ѹ��ֵ
	if (pressure_handle->pressure_adc > 8388608) //���� 8388606 Ϊ��ѹֵ�� ������ʾ�ն��������Ŵ���
	{
		pressure_handle->pressure = (pressure_handle->pressure_adc - 16777216) / 8;	//��λΪ Pa
	}
	else
	{
		pressure_handle->pressure = pressure_handle->pressure_adc / 8;	//��λΪ Pa
	}
	//Read ADC output data of temperature ��ȡ�����¶�ֵ�� 2 ���Ĵ�����ֵ
	pressure_handle->temperature_H = Read_One_Byte(0x09);
	pressure_handle->temperature_L = Read_One_Byte(0x0A);
	//Compute the value of temperature converted by ADC ���㴫���� ADC ת�����ѹ���¶�ֵ
	pressure_handle->temperature_adc = pressure_handle->temperature_H * 256 + pressure_handle->temperature_L;
	//The conversion formula of calibrated temperature, its unit is Centigrade ��������У׼����¶�ֵ
	if (pressure_handle->temperature_adc > 32768)
		pressure_handle->temperature = (pressure_handle->temperature_adc - 65536) / 256; //��λΪ���϶�
	else
		pressure_handle->temperature = pressure_handle->temperature_adc / 256; //��λΪ���϶�
//	HAL_Delay(100);
}






