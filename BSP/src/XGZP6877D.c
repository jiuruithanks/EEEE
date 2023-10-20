/**
 * @file XGZP6877D.c
 * @author Zhao Haofei
 * @brief 压力传感器驱动
 * @version 1.0
 * @date 2023-07-24
 *
 * @copyright Copyright (c) 2023
 */
#include "XGZP6877D.h"


XGZP6877D_HANDLE xgzp6877d_handle;


/*********************************************************XGZP6877D驱动***********************************************************************/ 

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
	/*0x30 里写入测量命令， 000： 单次温度测量； 001： 单次压力测量； 010： 组合： 单次压力和温度
测量； 011： 休眠方式（以一定的时间间隔执行组合模式测量）*/
	Write_One_Byte(0x30, 0x0A);
	//Judge whether Data collection is over 判断数据采集是否结束
	while ((Read_One_Byte(0x30) & 0x08) > 0);
	HAL_Delay(20);
	// Read ADC output Data of Pressure 读取保存压力值的 3 个寄存器的值
	pressure_handle->pressure_H = Read_One_Byte(0x06);
	pressure_handle->pressure_M = Read_One_Byte(0x07);
	pressure_handle->pressure_L = Read_One_Byte(0x08);
	//Compute the value of pressure converted by ADC 计算传感器 ADC 转换后的压力值
	pressure_handle->pressure_adc = pressure_handle->pressure_H * 65536 + pressure_handle->pressure_M * 256 + pressure_handle->pressure_L;
	//The conversion formula of calibrated pressure， its unit is Pa 计算最终校准后的压力值
	if (pressure_handle->pressure_adc > 8388608) //超过 8388606 为负压值， 需在显示终端做正负号处理
	{
		pressure_handle->pressure = (pressure_handle->pressure_adc - 16777216) / 8;	//单位为 Pa
	}
	else
	{
		pressure_handle->pressure = pressure_handle->pressure_adc / 8;	//单位为 Pa
	}
	//Read ADC output data of temperature 读取保存温度值的 2 个寄存器的值
	pressure_handle->temperature_H = Read_One_Byte(0x09);
	pressure_handle->temperature_L = Read_One_Byte(0x0A);
	//Compute the value of temperature converted by ADC 计算传感器 ADC 转换后的压力温度值
	pressure_handle->temperature_adc = pressure_handle->temperature_H * 256 + pressure_handle->temperature_L;
	//The conversion formula of calibrated temperature, its unit is Centigrade 计算最终校准后的温度值
	if (pressure_handle->temperature_adc > 32768)
		pressure_handle->temperature = (pressure_handle->temperature_adc - 65536) / 256; //单位为摄氏度
	else
		pressure_handle->temperature = pressure_handle->temperature_adc / 256; //单位为摄氏度
//	HAL_Delay(100);
}






