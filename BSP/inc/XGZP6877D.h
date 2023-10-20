#ifndef __XGZP6877D_H
#define __XGZP6877D_H

#include <stdint.h>
#include "myi2c.h"

/******************************************************* �궨�� **********************************************************/
/* �Ĵ�����ַ */
#define DATA_MSB 	0x06	//Pressure Data out<23:16>
#define DATA_CSB 	0x07	//Pressure Data out<15:8>
#define DATA_LSB 	0x08	//Pressure Data out<7:0>
#define TEMP_MSB 	0x09	//Temperature out<15:8>
#define TEMP_LSB 	0x0A	//Temperature out<7:0>
#define CMD			0x30	//Sleep_time<7:4> 	sco 			Measurement_ctrl<2:0> 
#define Sys_config	0xA5	//Aout_config<7:4> 	LDO_config 		Unipolar 	Data_out_c	Diag_on
#define P_config	0xA6	//Input Swap		Gain_P<5:3>		OSR_P<2:0>

/* cmd���� */
#define TEMP_SINGLE  0x00
#define PRESS_SINGLE 0x01

/******************************************************* �������� **********************************************************/
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
	uint8_t	 	  	  pressure_H;		//��ʱ������ ���ڱ���Ӵ������ж�������ѹ�����¶���صļĴ�������ֵ
	uint8_t           pressure_M;		
	uint8_t           pressure_L;		
	uint8_t			  temperature_H;			
	uint8_t			  temperature_L;
	uint32_t		  pressure_adc;		//��ʱ������ ���ڱ��洫���� ADC ת�����ѹ��ֵ���¶�ֵ
	uint32_t		  temperature_adc;
	double			  pressure;			//���ڱ���У׼���ѹ��ֵ���¶�ֵ
	double			  temperature;
}XGZP6877D_HANDLE;

/******************************************************* �ⲿ���� **********************************************************/
extern XGZP6877D_HANDLE xgzp6877d_handle;

/******************************************************* �������� **********************************************************/
void xgzp6877d_init(XGZP6877D_HANDLE *pressure_handle);
void pressure_measure_read(XGZP6877D_HANDLE *pressure_handle);

/******************************************************* ���IIC **********************************************************/
void Write_One_Byte(uint8_t addr, uint8_t data);
uint8_t Read_One_Byte(uint8_t addr);
void XGZP6877D_Init(void);
void Pressure_Temperature_Cal(XGZP6877D_HANDLE *pressure_handle);
#endif
