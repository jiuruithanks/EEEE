/*******************************************************************************
* Copyright 2018-2022 ALPS COMMUNICATION DEVICES TECHNOLOGY (SHANGHAI) CO., LTD.
* @author  Lv Qiwei
* @version v0.8.0
* @date    2022/06/27
* @brief   Here is a simple driver of HSPPADx4 series pressure sensors, 
*          including HSPPAD042A, 142A, 143A, 143C, 146A, 147A, and 148A.
* @note    Some macros, and IIC communication flow functions, may need 
*          modifications when being transplanted.
*******************************************************************************/

#include "hsppadx4.h"

//#include "delay.h"

/*Register Map*/
#define HSPPADX4_WIA   (0x00)
#define HSPPADX4_INFO  (0x01)
#define HSPPADX4_FFST  (0x02)
#define HSPPADX4_STAT  (0x03)
#define HSPPADX4_POUTL (0x04)
#define HSPPADX4_POUTM (0x05)
#define HSPPADX4_POUTH (0x06)
#define HSPPADX4_TOUTL (0x09)
#define HSPPADX4_TOUTH (0x0A)
#define HSPPADX4_ACTR  (0x0B)
#define HSPPADX4_STR   (0x0C)
#define HSPPADX4_DCTL  (0x0D)
#define HSPPADX4_CTL1  (0x0E)
#define HSPPADX4_CTL2  (0x0F)
#define HSPPADX4_ACTL1 (0x10)
#define HSPPADX4_ACTL2 (0x11)
#define HSPPADX4_FCTL  (0x12)
#define HSPPADX4_AVCL  (0x13)
#define HSPPADX4_I2CD  (0x15)
#define HSPPADX4_PNUM  (0x1C)
#define HSPPADX4_PDET  (0x20)
#define HSPPADX4_TDET  (0x22)
#define HSPPADX4_PSTC  (0x24)
#define HSPPADX4_SRST  (0x26)
#define HSPPADX4_ACTC  (0x28)
#define HSPPADX4_PTDET (0x29)

/*Constant Values*/
#define HSPPADX4_WIA_VALUE  (0x49)
#define HSPPADX4_INFO_VALUE (0x31)
#define HSPPADX4_ST_DEF     (0x55)
#define HSPPADX4_ST_OK      (0xAA)
#define HSPPADX4_LOWER_LIMIT      (30000)
#define HSPPADX4_UPPER_LIMIT_042A (110000)
#define HSPPADX4_UPPER_LIMIT_143A (210000)
#define HSPPADX4_UPPER_LIMIT_143C (200000)
#define HSPPADX4_UPPER_LIMIT_146A (1700000)
#define HSPPADX4_UPPER_LIMIT_147A (750000)
#define HSPPADX4_UPPER_LIMIT_148A (3200000)

/*Register Masks*/
#define HSPPADX4_FFEV_MASK     (0x80)
#define HSPPADX4_FP_MASK       (0x1F)
#define HSPPADX4_BUSY_MASK     (0x80)
#define HSPPADX4_TRDY_MASK     (0x10)
#define HSPPADX4_PRDY_MASK     (0x01)
#define HSPPADX4_PDRP_MASK     (0x10)
#define HSPPADX4_PTAP_MASK     (0x03)
#define HSPPADX4_TPMES_MASK    (0xA0)
#define HSPPADX4_ODR_MODE_MASK (0x0F)



/*Function Providing Delay of Several Milliseconds*/
#define DELAY_MS(nms) (Delay_ms(nms))

HSPPADX4_HANDLE  hsppad147_handle =
{
	&hi2c1,               						//I2C 接口
	(HSPPADX4_SADW)0x94,						//147地址8位地址，最后一位为0
	HSPPADX4_MODEL_147A,						//传感器型号
	{0},
	0,											///压力测量值
	0,											//温度数组
	0.0,										///温度值

};

HSPPADX4_CONFIGS hsppad147_config =
{

	HSPPADX4_MODE_CTN_100HZ,		//mode    配置为自动 100HZ
	HSPPADX4_FILTER_UA,				//超级高精度 
	1,
	1,
};



 /*******************************************************************************
 * @brief  This function makes a conversion to give the pressure data the correct 
 *         dimension (Pa).
 * @param  'pu8RegVal' is the address of raw output data of pressure.
 *         'eModel' indicates which HSPPADx4 series sensor that the reading comes 
 *         from.
 * @retval The pressure reading that has correct dimension.
 *******************************************************************************/
 static uint32_t hsppadx4_ConvertPressureReading(uint8_t *pu8RegVal, HSPPADX4_MODEL eModel)
 {
 	uint32_t u32Prs;
	
 	u32Prs = *pu8RegVal | ((uint_fast16_t)*(pu8RegVal+1) << 8) | ((uint_fast32_t)*(pu8RegVal+2) << 16);
 	if(eModel == HSPPADX4_MODEL_143C)
 		return u32Prs << 1;
 	if(eModel == HSPPADX4_MODEL_147A)
 		return u32Prs * 6;
 	if(eModel == HSPPADX4_MODEL_148A)
 		return u32Prs * 25;
 	if(eModel == HSPPADX4_MODEL_143A || eModel == HSPPADX4_MODEL_142A)
 		return u32Prs << 1;
 	if(eModel == HSPPADX4_MODEL_146A)
 		return u32Prs * 13;
 	return u32Prs;
 }

uint8_t Reg_test[6];
HAL_StatusTypeDef ret;
/*******************************************************************************
* @brief  This function gets pressure data from HSPPADx4 sensor.
* @param  'pstHandle' is the address of the structure that represents a sensor 
*         entity.
* @retval An errorcode. The meanings are listed below:
*         0x00  No error.
*         0x01  PRDY flag was not set. Old data were read.
*         0x02  Input 'pstHandle' is invalid.
*         0x08  Failed to communicate with sensor.
*******************************************************************************/
uint8_t bsp_hsppadx4_GetPressureInPa(HSPPADX4_HANDLE *pstHandle)
{
	HSPPADX4_SADW eSAdW;
	uint8_t au8RegVal[6];
//	uint_fast8_t uf8PollCnt;

	/*Check parameter.*/
	if(!pstHandle)
		return 0x02;
	
	/*Load address.*/
	eSAdW = pstHandle->m_SAdW;
	
	/*Try to read new pressure data.*/

	
		ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_STAT, 1, &au8RegVal[0], 4, HAL_MAX_DELAY);
		HAL_Delay(1);
/* 		if (ret != HAL_OK) return ret;
	
			ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_POUTL, 1, &au8RegVal[1], 1, HAL_MAX_DELAY);
		HAL_Delay(1);
		if (ret != HAL_OK) return ret;
			ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_POUTM, 1, &au8RegVal[2], 1, HAL_MAX_DELAY);
		HAL_Delay(1);
		if (ret != HAL_OK) return ret;
			ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_POUTH, 1, &au8RegVal[3], 1, HAL_MAX_DELAY);
		HAL_Delay(1);
		if (ret != HAL_OK) return ret; */

			ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_TOUTL, 1, &au8RegVal[4], 2, HAL_MAX_DELAY);
		HAL_Delay(1);
/* 			ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_TOUTH, 1, &au8RegVal[5], 1, HAL_MAX_DELAY);
		HAL_Delay(1); */

	for (uint8_t i=0;i<4;i++)
	{
	Reg_test[i]=au8RegVal[i];
	
	pstHandle->m_pressure_h[i] = &au8RegVal[i];
	};
	pstHandle->m_temp_h=(au8RegVal[5]<<8)|au8RegVal[4];
	pstHandle->m_temp_f=pstHandle->m_temp_h/256.0;
	/*Form pressure data.*/
	pstHandle->m_Pressure = hsppadx4_ConvertPressureReading(au8RegVal+1, pstHandle->m_Model);
	
	/*Return errorcode if PRDY==0 and the pressure data is not the latest.*/
	return 0x01;
}

_Bool bsp_hsppadx4_Init(HSPPADX4_HANDLE *pstHandle, HSPPADX4_CONFIGS *pstInitConfig)
{
	HSPPADX4_SADW eSAdW;
	uint8_t u8ConfVal, u8RegVal;
	HAL_StatusTypeDef ret;
	/*Check parameters.*/
	if(!(pstHandle && pstInitConfig))
		return 0;
	/*Load address.*/
	eSAdW = pstHandle->m_SAdW;
	/*Set pressure measurement accuracy (and also power consumption).*/
	u8ConfVal = pstInitConfig->m_FilterConfig;
	u8ConfVal &= HSPPADX4_PTAP_MASK;//Deal with illegal values.
	u8ConfVal |= HSPPADX4_PDRP_MASK;

	//u8ConfVal=0x03;					//超高精度
	ret = HAL_I2C_Mem_Write(pstHandle->HI2C, eSAdW, HSPPADX4_CTL1, 1, &u8ConfVal, 1, HAL_MAX_DELAY);
	HAL_Delay(1);
	if (ret != HAL_OK) return ret;
	ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_CTL1, 1, &u8RegVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;

	if(u8RegVal!=u8ConfVal)
		return 0;
	
	
	/*Configure FIFO.*/
	u8ConfVal = pstInitConfig->m_FifoConfig ? 0x90 : 0x10;    //开启FIFO

	ret = HAL_I2C_Mem_Write(pstHandle->HI2C, eSAdW, HSPPADX4_FCTL, 1, &u8ConfVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;
	
	ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_FCTL, 1, &u8RegVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;

	if(u8RegVal!=u8ConfVal)
		return 0;
	/*Configure averaging.*/
	u8ConfVal = pstInitConfig->m_AvgConfig ? 0x24 : 0x38;        //平均次数

	ret = HAL_I2C_Mem_Write(pstHandle->HI2C, eSAdW, HSPPADX4_AVCL, 1,  &u8ConfVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;
	
	ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_AVCL, 1, &u8RegVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;

	if(u8RegVal!=u8ConfVal)
		return 0;
//	
	/*Set measurement mode.*/
	u8ConfVal = pstInitConfig->m_ModeConfig;
	u8ConfVal &= HSPPADX4_ODR_MODE_MASK;
	u8ConfVal |= HSPPADX4_TPMES_MASK;
	//u8ConfVal=0x29;//只测气压bit5， 频率100HZ  bit3bit2  10   连续测量模式  bit1bit0  01;
	//u8ConfVal=0xA9;// 温度测量bit 7  气压bit5， 频率100HZ  bit3bit2  10   连续测量模式  bit1bit0  01;

	ret = HAL_I2C_Mem_Write(pstHandle->HI2C, eSAdW, HSPPADX4_CTL2, 1,  &u8ConfVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;

	ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_CTL2, 1, &u8RegVal, 1, HAL_MAX_DELAY);
		HAL_Delay(1);
	if (ret != HAL_OK) return ret;
	return u8RegVal==u8ConfVal;
}

// static _Bool hsppadx4_MultiBytesReadFlow(HSPPADX4_SADW eSAdW, uint8_t u8Reg, uint8_t *pu8Data, uint_fast8_t uf8Len)
// /*******************************************************************************
// * @brief  The IIC reading flow, which can read multiple bytes.
// * @note   The format here is programmed according to the sensor datasheet.
// * @param  'eSAdW' is the IIC slave address with the R/W bit set to 'write'.
// *         'u8Reg' is the address of the first register to be read.
// *         'pu8Data' is the address of data recording array (or a simple variable).
// *         'uf8Len' is the number of data bytes to be read.
// * @retval A boolean indicates if the reading flow is successfully completed.
// *         Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	_Bool bCompleted = 0;
	
// 	if(iic_Start())//S (Comments here are same as the terms in the sensor datasheet.)
// 	{
// 		if(iic_SendByte((uint8_t)eSAdW))//SAD+W, SAK
// 		{
// 			if(iic_SendByte(u8Reg))//REG, SAK
// 			{
// 				if(iic_Start())//SR
// 				{
// 					if(iic_SendByte((uint8_t)eSAdW | 0x01))//SAD+R, SAK
// 					{
// 						bCompleted = iic_ReadMultipleBytes(pu8Data, uf8Len);//DATA, A (or /A)
// 					}
// 				}
// 			}
// 		}
// 		bCompleted = iic_Stop() && bCompleted;//P
// 	}
// 	return bCompleted;
// }

// static _Bool hsppadx4_OneByteWriteFlow(HSPPADX4_SADW eSAdW, uint8_t u8Reg, uint8_t u8Data)
// /*******************************************************************************
// * @brief  The IIC writing flow.
// * @note   The format here is programmed according to the sensor datasheet.
// * @param  'eSAdW' is the IIC slave address with the R/W bit set to 'write'.
// *         'u8Reg' is the address of the register to be written.
// *         'u8Data' is the data byte to be written to the register.
// * @retval A boolean indicates if the writing flow is successfully completed.
// *         Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	_Bool bCompleted = 0;
	
// 	if(iic_Start())//S (Comments here are same as the terms in the sensor datasheet.)
// 	{
// 		if(iic_SendByte((uint8_t)eSAdW))//SAD+W, SAK
// 		{
// 			if(iic_SendByte(u8Reg))//REG, SAK
// 			{
// 				bCompleted = iic_SendByte(u8Data);//DATA, SAK
// 			}
// 		}
// 		bCompleted = iic_Stop() && bCompleted;//P
// 	}
// 	return bCompleted;
// }

//static _Bool hsppadx4_ActionCommandFlow(HSPPADX4_SADW eSAdW, uint8_t u8Reg)
///*******************************************************************************
//* @brief  The action command flow.
//* @note   The format here is programmed as the action command. It may not be 
//*         introduced in the sensor datasheet, but actually used in command mode.
//*         It just seems like a writing flow, which stops before delivering the 
//*         data to be written. Therefore, no transmission error during this flow 
//*         can change the value of registers.
//* @param  'eSAdW' is the IIC slave address with the R/W bit set to 'write'.
//*         'u8Reg' is the address of the command register.
//* @retval A boolean indicates if the command flow is successfully completed.
//*         Value 1 means success, 0 means fail.
//*******************************************************************************/
//{
//	_Bool bCompleted = 0;
//	
//	if(iic_Start())//S (Comments here are same as the terms in the sensor datasheet.)
//	{
//		if(iic_SendByte((uint8_t)eSAdW))//SAD+W, SAK
//		{
//			bCompleted = iic_SendByte(u8Reg);//REG, SAK
//		}
//		bCompleted = iic_Stop() && bCompleted;//P
//	}


//	
//	return bCompleted;
//}

// _Bool hsppadx4_CheckExistence(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function checks the existence of HSPPADx4 sensor and its model.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// *         Its member 'm_Model' indicates which HSPPADx4 series sensor it is.
// *         The model of sensor should be set correctly before calling this 
// *         function. Set to HSPPADX4_MODEL_NULL if the model is unknown.
// * @retval A boolean indicates if the sensor exists and fits the given model.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t u8RegVal;
	
// 	/*Check parameter.*/
// 	if(!pstHandle)
// 		return 0;
	
// 	/*Time for Sensor Power-On Reset*/
// 	DELAY_MS(3);
	
// 	/*Find the correct address of sensor.*/
// 	do
// 	{
// 		eSAdW = HSPPADX4_SADW_GENERIC;
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_PNUM, &u8RegVal, 1))
// 			break;
// 		eSAdW = HSPPADX4_SADW_147A;
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_PNUM, &u8RegVal, 1))
// 			break;
// 		eSAdW = HSPPADX4_SADW_148A;
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_PNUM, &u8RegVal, 1))
// 			break;
// 		return 0;
// 	}while(0);
// 	pstHandle->m_SAdW = eSAdW;
	
// 	/*Check product number (if it is given).*/
// 	if(pstHandle->m_Model == HSPPADX4_MODEL_NULL)
// 	{
// 		pstHandle->m_Model = (HSPPADX4_MODEL)u8RegVal;
// 	}
// 	else
// 	{
// 		if(u8RegVal != (uint8_t)pstHandle->m_Model)
// 			return 0;
// 	}
	
// 	/*Check 'Who I Am'.*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_WIA, &u8RegVal, 1))
// 		return 0;
// 	if(u8RegVal != HSPPADX4_WIA_VALUE)
// 		return 0;
	
// 	/*Check INFO register.*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_INFO, &u8RegVal, 1))
// 		return 0;
// 	if(u8RegVal != HSPPADX4_INFO_VALUE)
// 		return 0;
	
// 	return 1;
// }

// static uint8_t hsppadx4_RunSelfTestByCmd(HSPPADX4_SADW eSAdW)
// /*******************************************************************************
// * @brief  This function makes HSPPADx4 sensor to run self-tests in command 
// *         action mode.
// * @note   Access Check Test is the test of communication and register 
// *         accessibility.
// *         Self Test of Pressure Measurement is the test of pressure transmitters 
// *         and AD converter.
// *         Note that there might be 'normal' pressure readings even if there are 
// *         malfunctions in fact.
// *         Also note that wrong temperature readings could lead to wrong pressure 
// *         readings, even if the sensor has passed self-tests.
// * @param  'eSAdW' is the IIC slave address with the R/W bit set to 'write'.
// * @retval The meaning of this byte is listed below:
// *         0x00  Tests passed, no error.
// *         0x01  The register value was abnormal before Access Check Test.
// *         0x02  The register value was abnormal during Access Check Test.
// *         0x03  The register value was abnormal after Access Check Test.
// *         0x04  Failed at the second step of Pressure Measurement Test.
// *         0x08  Failed at the third step of Pressure Measurement Test.
// *         0x10  The register value was abnormal before Pressure Measurement Test.
// *         0x20  The register value was abnormal after Pressure Measurement Test.
// *         0x55  Pressure Measurement Test did not start as expected.
// *         0xF0  Failed at the first step of Pressure Measurement Test.
// *         0xFF  Failed to communicate with sensor.
// *******************************************************************************/
// {
// 	uint8_t u8RegVal;
	
// 	/*Access Check Test*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_ACTR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_DEF)
// 		return 0x01;
// 	if(!hsppadx4_ActionCommandFlow(eSAdW, HSPPADX4_ACTC))
// 		return 0xFF;
// 	DELAY_MS(2);
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_ACTR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_OK)
// 		return 0x02;
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_ACTR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_DEF)
// 		return 0x03;
	
// 	/*Self Test of Pressure Measurement*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_DEF)
// 		return 0x10;
// 	if(!hsppadx4_ActionCommandFlow(eSAdW, HSPPADX4_PSTC))
// 		return 0xFF;
// 	DELAY_MS(5);
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_OK)
// 		return u8RegVal;
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STR, &u8RegVal, 1) || u8RegVal!=HSPPADX4_ST_DEF)
// 		return 0x20;
	
// 	return 0x00;
// }

// uint8_t hsppadx4_SelfTestByCmd(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function sets HSPPADx4 sensor to command action mode temporarily, 
// *         and makes the sensor to run self-tests.
// * @note   There might be 'normal' pressure readings even if there are 
// *         malfunctions in fact. Therefore, self-tests are indispensable for end 
// *         users.
// *         Also note that wrong temperature readings could lead to wrong pressure 
// *         readings, even if the sensor has passed self-tests.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval The conclusion of self-tests. 0 means passed. Otherwise the value is 
// *         an errorcode. Please check the annotation of the function above. 
// *         Besides, 0xFC means that 'pstHandle' is invalid.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t u8RegVal, u8ErrCode;
	
// 	/*Check parameter.*/
// 	if(!pstHandle)
// 		return 0xFC;
	
// 	/*Load address.*/
// 	eSAdW = pstHandle->m_SAdW;
	
// 	/*Set to command mode.*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_CTL2, &u8RegVal, 1))
// 		return 0xFF;
// 	hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_CTL2, 0xA2);
	
// 	/*Run self-tests.*/
// 	u8ErrCode = hsppadx4_RunSelfTestByCmd(eSAdW);
	
// 	/*Return to the former mode.*/
// 	hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_CTL2, u8RegVal);
	
// 	return u8ErrCode;
// }

// _Bool hsppadx4_Init(HSPPADX4_HANDLE *pstHandle, HSPPADX4_CONFIGS *pstInitConfig)
// /*******************************************************************************
// * @brief  This function sets the action mode of HSPPADx4 sensor.
// * @note   The FIFO capacity is 16 bytes. Average is made with 16 pressure data.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// *         'pstInitConfig' is the address of the structure that contains user 
// *         configuration.
// * @retval A boolean indicates if the control register writing flow is 
// *         successfully done. Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t u8ConfVal, u8RegVal;
	
// 	/*Check parameters.*/
// 	if(!(pstHandle && pstInitConfig))
// 		return 0;
	
// 	/*Load address.*/
// 	eSAdW = pstHandle->m_SAdW;
	
// 	/*Set pressure measurement accuracy (and also power consumption).*/
// 	u8ConfVal = pstInitConfig->m_FilterConfig;
// 	u8ConfVal &= HSPPADX4_PTAP_MASK;//Deal with illegal values.
// 	u8ConfVal |= HSPPADX4_PDRP_MASK;
	
// 	if(!hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_CTL1, u8ConfVal))
// 		return 0;
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_CTL1, &u8RegVal, 1))
// 		return 0;
// 	if(u8RegVal!=u8ConfVal)
// 		return 0;
	
// 	/*Configure FIFO.*/
// 	u8ConfVal = pstInitConfig->m_FifoConfig ? 0x90 : 0x10;
// 	if(!hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_FCTL, u8ConfVal))
// 		return 0;
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_FCTL, &u8RegVal, 1))
// 		return 0;
// 	if(u8RegVal!=u8ConfVal)
// 		return 0;
	
// 	/*Configure averaging.*/
// 	u8ConfVal = pstInitConfig->m_AvgConfig ? 0x24 : 0x38;
// 	if(!hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_AVCL, u8ConfVal))
// 		return 0;
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_AVCL, &u8RegVal, 1))
// 		return 0;
// 	if(u8RegVal!=u8ConfVal)
// 		return 0;
	
// 	/*Set measurement mode.*/
// 	u8ConfVal = pstInitConfig->m_ModeConfig;
// 	u8ConfVal &= HSPPADX4_ODR_MODE_MASK;
// 	u8ConfVal |= HSPPADX4_TPMES_MASK;
// 	if(!hsppadx4_OneByteWriteFlow(eSAdW, HSPPADX4_CTL2, u8ConfVal))
// 		return 0;
// 	DELAY_MS(1);//Wait for mode switching.
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_CTL2, &u8RegVal, 1))
// 		return 0;
// 	return u8RegVal==u8ConfVal;
// }





// _Bool hsppadx4_SoftReset(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function makes HSPPADx4 sensor to execute a soft reset.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval A boolean indicates if the command is send successfully.
// *         Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	if(!pstHandle)
// 		return 0;
// 	if(!hsppadx4_OneByteWriteFlow(pstHandle->m_SAdW, HSPPADX4_ACTL2, 0x80))//Set software reset bit.
// 		if(!hsppadx4_OneByteWriteFlow(pstHandle->m_SAdW, HSPPADX4_ACTL2, 0x80))
// 			return 0;
// 	DELAY_MS(3);//Wait for reset completion.
// 	return 1;
// }

// _Bool hsppadx4_StartMeasurementByReg(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function writes specific registers of HSPPADx4 sensor to let it 
// *         measure temperature and pressure once.
// * @note   The sensor should be set into register action mode previously.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval A boolean indicates if the command is send successfully.
// *         Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	return pstHandle && hsppadx4_OneByteWriteFlow(pstHandle->m_SAdW, HSPPADX4_ACTL1, 0x0A);
// }

// _Bool hsppadx4_StartMeasurementByCmd(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function commands HSPPADx4 sensor to measure temperature and 
// *         pressure once.
// * @note   The sensor should be set into command action mode previously.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval A boolean indicates if the command is send successfully.
// *         Value 1 means success, 0 means fail.
// *******************************************************************************/
// {
// 	return pstHandle && hsppadx4_ActionCommandFlow(pstHandle->m_SAdW, HSPPADX4_PTDET);
// }
//_Bool bsp_hsppadx4_StartMeasurementByCmd(HSPPADX4_HANDLE *pstHandle)
//{
//		HAL_StatusTypeDef ret;
//	ret =HAL_I2C_Master_Transmit(pstHandle->HI2C, pstHandle->m_SAdW, HSPPADX4_PTDET, 1,HAL_MAX_DELAY);
//	if (ret != HAL_OK) return ret;


//}


//_Bool hsppadx4_CheckStandby(HSPPADX4_HANDLE *pstHandle)
///*******************************************************************************
//* @brief  This function checks if the sensor has finished its work.
//* @note   It is recommended to give sufficient time for the measurement before 
//*         calling this function.
//*         Please refer to section 8.3 in the sensor manual to get the length of 
//*         this timespan.
//* @param  'pstHandle' is the address of the structure that represents a sensor 
//*         entity.
//* @retval A boolean indicates if the measurement is done.
//*         Value 1 means success, 0 means error.
//*******************************************************************************/
//{
//	HSPPADX4_SADW eSAdW;
//	uint8_t u8RegVal;
//	uint_fast8_t uf8PollCnt;
//		HAL_StatusTypeDef ret;
//	if(!pstHandle)
//		return 0;
//	eSAdW = pstHandle->m_SAdW;
//	uf8PollCnt = 137;
//	do
//	{
//		ret = HAL_I2C_Mem_Read(pstHandle->HI2C, eSAdW, HSPPADX4_STAT, 1, &u8RegVal, 1, HAL_MAX_DELAY);
//	if (ret != HAL_OK) return ret;

//		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STAT, &u8RegVal, 1))
//			if(!(u8RegVal & HSPPADX4_BUSY_MASK))
//				return 1;
//		DELAY_MS(2);
//	}while(--uf8PollCnt);
//	return 0;
//}



// uint8_t hsppadx4_GetPressureInPa(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function gets pressure data from HSPPADx4 sensor.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval An errorcode. The meanings are listed below:
// *         0x00  No error.
// *         0x01  PRDY flag was not set. Old data were read.
// *         0x02  Input 'pstHandle' is invalid.
// *         0x08  Failed to communicate with sensor.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t au8RegVal[4];
// 	uint_fast8_t uf8PollCnt;
	
// 	/*Check parameter.*/
// 	if(!pstHandle)
// 		return 0x02;
	
// 	/*Load address.*/
// 	eSAdW = pstHandle->m_SAdW;
	
// 	/*Try to read new pressure data.*/
// 	uf8PollCnt = 8;
// 	do
// 	{
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STAT, au8RegVal, 4))
// 			if(au8RegVal[0] & HSPPADX4_PRDY_MASK)
// 				break;
// 		DELAY_MS(1);
// 	}while(--uf8PollCnt);
	
// 	/*Form pressure data.*/
// 	pstHandle->m_Pressure = hsppadx4_ConvertPressureReading(au8RegVal+1, pstHandle->m_Model);
	
// 	/*Return errorcode if PRDY==0 and the pressure data is not the latest.*/
// 	if(uf8PollCnt)
// 		return 0;
// 	else
// 		return 0x01;
// }


// uint8_t hsppadx4_GetTemperatureInLsb(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function gets raw temperature data from HSPPADx4 sensor.
// * @note   The dimension is 0.00390625 (=1/256) Celsius degree.
// *         Therefore, if the dimension is Celsius degree, the higher byte is just 
// *         the integral part, and the lower byte is the decimal part.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval An errorcode. The meanings are listed below:
// *         0x00  No error.
// *         0x02  Input 'pstHandle' is invalid.
// *         0x08  Failed to communicate with sensor.
// *         0x10  TRDY flag was not set. Old data were read.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t au8RegVal[2];
// 	uint_fast8_t uf8PollCnt;
	
// 	/*Check parameter.*/
// 	if(!pstHandle)
// 		return 0x02;
	
// 	/*Load address.*/
// 	eSAdW = pstHandle->m_SAdW;
	
// 	/*Check if there are new temperature data to read.*/
// 	uf8PollCnt = 4;
// 	do
// 	{
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_STAT, au8RegVal, 1))
// 			if(au8RegVal[0] & HSPPADX4_TRDY_MASK)
// 				break;
// 		DELAY_MS(1);
// 	}while(--uf8PollCnt);
	
// 	/*Get temperature data.*/
// 	if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_TOUTL, au8RegVal, 2))
// 		return 0x08;
// 	pstHandle->m_RawTemp = (int16_t)(au8RegVal[0] | ((uint_fast16_t)au8RegVal[1] << 8));
	
// 	/*Return errorcode if TRDY==0 and the temperature data is not the latest.*/
// 	if(uf8PollCnt)
// 		return 0;
// 	else
// 		return 0x10;
// }

// float hsppadx4_ConvertTemperatureToCelsiusDegree(int16_t s16RawTemp)
// /*******************************************************************************
// * @brief  This function converts raw temperature data to float type with Unit of 
// *         Celsius Degree.
// * @param  's16RawTemp' is the raw temperature data.
// * @retval The converted data.
// *******************************************************************************/
// {
// 	return (float)s16RawTemp * 0.00390625f;
// }

// uint8_t hsppadx4_GetReadingsByOneRead(HSPPADX4_HANDLE *pstHandle, _Bool bTempRead)
// /*******************************************************************************
// * @brief  This function gets status, pressure and temperature data from HSPPADx4 
// *         sensor by one multi-bytes IIC read.
// * @note   This function is designed for users who want to reduce time used for 
// *         reading data. Therefore, the function has only one IIC read flow, and 
// *         does not do polling, to let users control delay time.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// *         'bTempRead' indicates if temperature is to be read.
// * @retval An errorcode. If it is 0x00, it means 'no error'. If any anomaly 
// *         exists, a flag bit will be set.
// *         There are multiple kinds of anomaly that may exist, and their flag 
// *         bits are ANDed together.
// *         The flags are listed below:
// *         0x01  PRDY flag was not set. Old pressure data were read. Does not 
// *               affect the validity of temperature data.
// *         0x02  Input 'pstHandle' is invalid.
// *         0x08  Failed to communicate with sensor.
// *         0x10  TRDY flag was not set. Old temperature data were read. Does not 
// *               affect the validity of pressure data.
// *         0x80  BUSY flag was still set. Old data were read.
// *******************************************************************************/
// {
// 	uint8_t au8RegVal[8], u8RetVal;
// 	uint_fast8_t uf8ReadLen;
	
// 	/*Check parameter.*/
// 	if(!pstHandle)
// 		return 0x02;
	
// 	/*Read from register STAT to TOUTH (or POUTH).*/
// 	if(bTempRead)
// 		uf8ReadLen = 8;
// 	else
// 		uf8ReadLen = 4;
// 	if(!hsppadx4_MultiBytesReadFlow(pstHandle->m_SAdW, HSPPADX4_STAT, au8RegVal, uf8ReadLen))
// 		return 0x08;
	
// 	/*Form pressure data.*/
// 	pstHandle->m_Pressure = hsppadx4_ConvertPressureReading(au8RegVal+1, pstHandle->m_Model);
	
// 	/*Form temperature data.*/
// 	if(bTempRead)
// 		pstHandle->m_RawTemp = (int16_t)(au8RegVal[6] | ((uint_fast16_t)au8RegVal[7] << 8));
	
// 	/*Check STAT and form errorcode.*/
// 	u8RetVal = au8RegVal[0] & HSPPADX4_BUSY_MASK;
// 	if(!(au8RegVal[0] & HSPPADX4_TRDY_MASK))
// 		u8RetVal |= HSPPADX4_TRDY_MASK;
// 	if(!(au8RegVal[0] & HSPPADX4_PRDY_MASK))
// 		u8RetVal |= HSPPADX4_PRDY_MASK;
// 	return u8RetVal;
// }

// _Bool hsppadx4_CheckFifoFull(HSPPADX4_HANDLE *pstHandle)
// /*******************************************************************************
// * @brief  This function checks if the FIFO is full.
// * @note   Use this only when FIFO is enabled.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// * @retval A boolean indicates if the FIFO is full.
// *         Communication errors could also make this function return zero.
// *******************************************************************************/
// {
// 	uint8_t u8RegVal;
	
// 	if(!hsppadx4_MultiBytesReadFlow(pstHandle->m_SAdW, HSPPADX4_FFST, &u8RegVal, 1))
// 		if(!hsppadx4_MultiBytesReadFlow(pstHandle->m_SAdW, HSPPADX4_FFST, &u8RegVal, 1))
// 			return 0;
// 	return u8RegVal & HSPPADX4_FFEV_MASK;
// }

// uint_fast8_t hsppadx4_ReadFifo(HSPPADX4_HANDLE *pstHandle, uint32_t *pu32Data, uint_fast8_t uf8MaxLen)
// /*******************************************************************************
// * @brief  This function reads data from FIFO continually, until FIFO is empty, 
// *         or the data recording array is full.
// * @note   Use this only when FIFO is enabled.
// * @param  'pstHandle' is the address of the structure that represents a sensor 
// *         entity.
// *         'pu32Data' is the address of pressure data recording array.
// *         'uf8MaxLen' is the maximum length of the array.
// * @retval The number of pressure data recorded by the array. It is never bigger 
// *         than the array length, or the number of data in FIFO.
// *******************************************************************************/
// {
// 	HSPPADX4_SADW eSAdW;
// 	uint8_t au8RegVal[3];
// 	uint_fast8_t uf8ReadCnt, uf8PollCnt;
	
// 	if(!(pstHandle && pu32Data))
// 		return 0;
// 	eSAdW = pstHandle->m_SAdW;
// 	uf8ReadCnt = 0;
// 	uf8PollCnt = 23;
// 	do
// 	{
// 		if(!hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_FFST, au8RegVal, 1))
// 			continue;
// 		if(!(*au8RegVal & HSPPADX4_FP_MASK))//Check if there is any datum in FIFO.
// 			break;
// 		if(hsppadx4_MultiBytesReadFlow(eSAdW, HSPPADX4_POUTL, au8RegVal, 3))//Read data out.
// 		{
// 			*pu32Data = hsppadx4_ConvertPressureReading(au8RegVal, pstHandle->m_Model);
// 			pu32Data++;
// 			uf8ReadCnt++;
// 		}
// 	}while(uf8ReadCnt<uf8MaxLen && --uf8PollCnt);
// 	return uf8ReadCnt;
// }

/*Test Mode Functions*/
#if _DEBUG_HSPPADX4_TESTMODE
#include "hsppadx4_testmode.c"
#endif
