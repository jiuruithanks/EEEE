/*******************************************************************************
* Copyright 2018-2022 ALPS COMMUNICATION DEVICES TECHNOLOGY (SHANGHAI) CO., LTD.
* @author  Lv Qiwei
* @version v0.8.0
* @date    2022/04/28
* @brief   This is a head file for HSPPADx4 series pressure sensors driver.
*          This head file defines all register address, register bit masks, and 
*          other permanent values of HSPPADx4 series pressure sensors.
*          All 'public' functions of the driver are declared here as well.
*          Note that some includes and macros may need modifications when being 
*          transplanted.
*******************************************************************************/

#ifndef _HSPPADX4_H
#define _HSPPADX4_H

/*This head file is included to give definitions to data types like 'uint8_t'.*/
#include <stdint.h>

//#include "i2c.h"

/*Test Mode Control*/
#define _DEBUG_HSPPADX4_TESTMODE 0 //Zero for End Users

/*Sensor Model Enumeration (According to Product Numbers)*/
typedef enum
{
	HSPPADX4_MODEL_NULL = 0x00,
	HSPPADX4_MODEL_042A = 0x40,
	HSPPADX4_MODEL_141A = 0x90,
	HSPPADX4_MODEL_142A = 0xA0,
	HSPPADX4_MODEL_143A = 0xE0,
	HSPPADX4_MODEL_143C = 0x61,
	HSPPADX4_MODEL_145A = 0x11,
	HSPPADX4_MODEL_146A = 0x21,
	HSPPADX4_MODEL_147A = 0x31,
	HSPPADX4_MODEL_148A = 0x41
}HSPPADX4_MODEL;

/*IIC Slave Address Enumeration*/
typedef enum
{
	HSPPADX4_SADW_GENERIC = 0x90,
	HSPPADX4_SADW_147A    = 0x94,
	HSPPADX4_SADW_148A    = 0x92
}HSPPADX4_SADW;		//SADW = SAD + W : Slave Address + write bit

/*Action Mode Enumeration*/
typedef enum
{
  HSPPADX4_MODE_REG       = 0x00,//Register Action Mode
  HSPPADX4_MODE_CTN_1HZ   = 0x01,//Continuous Measurement Mode, 1Hz		Sampling rate
  HSPPADX4_MODE_CTN_10HZ  = 0x05,//Continuous Measurement Mode, 10Hz	Sampling rate
  HSPPADX4_MODE_CTN_100HZ = 0x09,//Continuous Measurement Mode, 100Hz	Sampling rate
  HSPPADX4_MODE_CTN_200HZ = 0x0D,//Continuous Measurement Mode, 200Hz	Sampling rate
  HSPPADX4_MODE_CMD       = 0x02 //Command Action Mode
}HSPPADX4_MODE;											

/*CIC Filter Configuration Enumeration*/
typedef enum
{
	HSPPADX4_FILTER_ULP = 0x00,//Ultra Low Power
	HSPPADX4_FILTER_LP  = 0x01,//Low Power
	HSPPADX4_FILTER_HA  = 0x02,//High Accuracy
	HSPPADX4_FILTER_UA  = 0x03 //Ultra Accuracy
}HSPPADX4_FILTER;

/*Sensor Handle*/
typedef struct
{
	I2C_HandleTypeDef *HI2C;
	HSPPADX4_SADW 	  m_SAdW;			//IIC Slave Address
	HSPPADX4_MODEL 	  m_Model;			
	uint8_t*	 	  m_pressure_h[4];	
	uint32_t          m_Pressure;		
	int16_t			  m_temp_h;			//temperature
	float			  m_temp_f;			
	
}HSPPADX4_HANDLE;

/*Configuration Structure*/
typedef struct
{
	HSPPADX4_MODE   m_ModeConfig;	//sensor mode
	HSPPADX4_FILTER m_FilterConfig;
	_Bool           m_FifoConfig;   //16 Step FIFO Control (Set as 1 to enable, 0 to disable.)
	_Bool           m_AvgConfig;    //16x Averaging Control (Set as 1 to enable, 0 to disable.)
}HSPPADX4_CONFIGS;



extern HSPPADX4_HANDLE  hsppad147_handle ;
extern HSPPADX4_CONFIGS hsppad147_config;


/*'Public' Functions*/
// _Bool hsppadx4_CheckExistence(HSPPADX4_HANDLE *pstHandle);
// uint8_t hsppadx4_SelfTestByCmd(HSPPADX4_HANDLE *pstHandle);
// _Bool hsppadx4_Init(HSPPADX4_HANDLE *pstHandle, HSPPADX4_CONFIGS *pstInitConfig);
// _Bool hsppadx4_SoftReset(HSPPADX4_HANDLE *pstHandle);
// _Bool hsppadx4_StartMeasurementByReg(HSPPADX4_HANDLE *pstHandle);
// _Bool hsppadx4_StartMeasurementByCmd(HSPPADX4_HANDLE *pstHandle);
// _Bool hsppadx4_CheckStandby(HSPPADX4_HANDLE *pstHandle);
// uint8_t hsppadx4_GetPressureInPa(HSPPADX4_HANDLE *pstHandle);
// uint8_t hsppadx4_GetTemperatureInLsb(HSPPADX4_HANDLE *pstHandle);
// float hsppadx4_ConvertTemperatureToCelsiusDegree(int16_t s16RawTemp);
// uint8_t hsppadx4_GetReadingsByOneRead(HSPPADX4_HANDLE *pstHandle, _Bool bTempRead);
// _Bool hsppadx4_CheckFifoFull(HSPPADX4_HANDLE *pstHandle);
// uint_fast8_t hsppadx4_ReadFifo(HSPPADX4_HANDLE *pstHandle, uint32_t *pu32Data, uint_fast8_t uf8MaxLen);

_Bool bsp_hsppadx4_Init(HSPPADX4_HANDLE *pstHandle, HSPPADX4_CONFIGS *pstInitConfig);
uint8_t bsp_hsppadx4_GetPressureInPa(HSPPADX4_HANDLE *pstHandle);
//uint8_t hsppadx4_GetTemperatureInLsb(HSPPADX4_HANDLE *pstHandle);
/*Test Mode Functions*/
#if _DEBUG_HSPPADX4_TESTMODE
#include "hsppadx4_testmode.h"
#endif

#endif /* _HSPPADX4_H */
