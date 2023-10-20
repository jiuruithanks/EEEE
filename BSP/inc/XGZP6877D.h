#ifndef __XGZP6877D_H
#define __XGZP6877D_H

#include <stdint.h>
#include "myi2c.h"

/******************************************************* 宏定义 **********************************************************/
/* 寄存器地址 */
#define DATA_MSB 	0x06	//Pressure Data out<23:16>
#define DATA_CSB 	0x07	//Pressure Data out<15:8>
#define DATA_LSB 	0x08	//Pressure Data out<7:0>
#define TEMP_MSB 	0x09	//Temperature out<15:8>
#define TEMP_LSB 	0x0A	//Temperature out<7:0>
#define CMD			0x30	//Sleep_time<7:4> 	sco 			Measurement_ctrl<2:0> 
#define Sys_config	0xA5	//Aout_config<7:4> 	LDO_config 		Unipolar 	Data_out_c	Diag_on
#define P_config	0xA6	//Input Swap		Gain_P<5:3>		OSR_P<2:0>

/* cmd命令 */
#define TEMP_SINGLE  0x00
#define PRESS_SINGLE 0x01

/******************************************************* 数据类型 **********************************************************/
typedef enum
{
	end_sco,
	work_sco
}XGZP6877D_FLAG;

typedef enum
{
	combined_collect = 0x02,
	sleep_work		 = 0x03
}XGZP6877D_MODEL;

typedef struct
{
	// I2C_HandleTypeDef *h2ic;
	uint8_t 	  	  slave;			//IIC Slave Address
	XGZP6877D_MODEL   model;			
	uint8_t	 	  	  pressure_H;		//临时变量， 用于保存从传感器中读出的与压力和温度相关的寄存器的数值
	uint8_t           pressure_M;		
	uint8_t           pressure_L;		
	uint8_t			  temperature_H;			
	uint8_t			  temperature_L;
	uint32_t		  pressure_adc;		//临时变量， 用于保存传感器 ADC 转换后的压力值和温度值
	uint32_t		  temperature_adc;
	double			  pressure;			//用于保存校准后的压力值和温度值
	double			  temperature;
}XGZP6877D_HANDLE;

/******************************************************* 外部声明 **********************************************************/
extern XGZP6877D_HANDLE xgzp6877d_handle;

/******************************************************* 函数声明 **********************************************************/
void xgzp6877d_init(XGZP6877D_HANDLE *pressure_handle);
void pressure_measure_read(XGZP6877D_HANDLE *pressure_handle);

/******************************************************* 软件IIC **********************************************************/
void Write_One_Byte(uint8_t addr, uint8_t data);
uint8_t Read_One_Byte(uint8_t addr);
void XGZP6877D_Init(void);
void Pressure_Temperature_Cal(XGZP6877D_HANDLE *pressure_handle);
#endif
