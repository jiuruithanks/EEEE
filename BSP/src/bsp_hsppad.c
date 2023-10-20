/*******************************************************************************
* @version	v0.6.3
* @date			2020/04/22
* @brief		Here is a simple driver of HSPPAD series pressure sensors, including 
* 					HSPPAD042A, 142A, 143A, 143C, 146A, and 147A.
*           Note that some external functions need modification when 
* 					transplanted.
*******************************************************************************/

#include "bsp_hsppad.h"

/*External Basic IIC Functions*/

/*External Delay Functions*/

/*******************************************************************************
* @brief	The IIC reading flow, which can read multiple bytes.
* @note		The format here is programmed according to the sensor datasheet.
* @param	'reg' is the address of the first register to be read.
* 				'dat' is the address of data recording array (or a simple variable).
* 				'len' is the munber of data bytes to be read.
* @retval	The number of data bytes which are read out via IIC. 0 means fail.
*******************************************************************************/
static uint8_t multi_bytes_read_flow(uint8_t reg, uint8_t *dat, uint8_t len)

{
	uint8_t flag = 0;
	if(IIC_START())//S (Comments here are same as the terms in the sensor datasheet.)
	{
		IIC_SEND(HSPPAD_SADW);//SAD+W
		if(IIC_WAIT_ACK())//SAK
		{
			#if IIC_LOGEN
			printf("ACK get.\r\n");
			#endif
			IIC_SEND(reg);//REG
			#if IIC_LOGEN
			printf("Register address %02X send.\r\n", reg);
			#endif
			if(IIC_WAIT_ACK())//SAK
			{
				#if IIC_LOGEN
				printf("ACK get.\r\n");
				#endif
				IIC_START();//SR
				IIC_SEND(HSPPAD_SADR);//SAD+R
				#if IIC_LOGEN
				printf("SADR send.\r\n");
				#endif
				if(IIC_WAIT_ACK())//SAK
				{
					while(len)
					{
						len--;
						*dat = IIC_READ(len);//DATA, A
						#if IIC_LOGEN
						printf("The byte is %02X.\r\n", *dat);
						#endif
						flag++;
						dat++;
					}
				}
			}
		}
		IIC_STOP();//P
	}
	return flag;
}

static uint8_t one_byte_write_flow(uint8_t reg, uint8_t dat)
/*******************************************************************************
* @brief	The IIC writing flow.
* @note		The format here is programmed according to the sensor datasheet.
* @param	'reg' is the address of the register to be written.
* 				'dat' is the data byte to be written to the register.
* @retval	A boolean indicates if the writing flow is successfully compeleted.
* 				Value 1 means success, 0 means fail.
*******************************************************************************/
{
	uint8_t flag = 0;
	
	if(IIC_START())//S (Comments here are same as the terms in the sensor datasheet.)
	{
		#if IIC_LOGEN
		printf("\r\nIIC Started.\r\n");
		#endif
		IIC_SEND(HSPPAD_SADW);//SAD+W
		#if IIC_LOGEN
		printf("\r\nSADW send.\r\n");
		#endif
		if(IIC_WAIT_ACK())//SAK
		{
			#if IIC_LOGEN
			printf("ACK get.\r\n");
			#endif
			IIC_SEND(reg);//REG
			#if IIC_LOGEN
			printf("Register address %02X send.\r\n", reg);
			#endif
			if(IIC_WAIT_ACK())//SAK
			{
				#if IIC_LOGEN
				printf("ACK get.\r\n");
				#endif
				IIC_SEND(dat);//DATA
				#if IIC_LOGEN
				printf("Data %02X send.\r\n", dat);
				#endif
				if(IIC_WAIT_ACK())//SAK
					flag++;
				#if IIC_LOGEN
				if(flag)
					printf("Success.\r\n");
				#endif
			}
		}
		IIC_STOP();//P
	}
	return flag;
}

static uint8_t action_command_flow(uint8_t reg)
/*******************************************************************************
* @brief	The action command flow.
* @note		The format here is programmed as the action command. It may not be 
* 				introduced in the sensor datasheet, but actually used in command mode.
* 				It just seems like a writing flow, which stops before delivering the 
* 				data to be written. Therefore, no transmition error during this flow 
* 				can change the value of registers.
* @param	'reg' is the address of the command register.
* @retval	A boolean indicates if the command flow is successfully compeleted.
* 				Value 1 means success, 0 means fail.
*******************************************************************************/
{
	uint8_t flag = 0;
	
	if(IIC_START())//S (Comments here are same as the terms in the sensor datasheet.)
	{
		#if IIC_LOGEN
		printf("\r\nIIC Started.\r\n");
		#endif
		IIC_SEND(HSPPAD_SADW);//SAD+W
		#if IIC_LOGEN
		printf("\r\nSADW send.\r\n");
		#endif
		if(IIC_WAIT_ACK())//SAK
		{
			#if IIC_LOGEN
			printf("ACK get.\r\n");
			#endif
			IIC_SEND(reg);//REG
			#if IIC_LOGEN
			printf("Register address %02X send.\r\n", reg);
			#endif
			if(IIC_WAIT_ACK())//SAK
				flag++;
			#if IIC_LOGEN
			if(flag)
				printf("Success.\r\n");
			#endif
		}
		IIC_STOP();//P
	}
	return flag;
}

HSPPAD_MODEL HSPPAD_Existence(HSPPAD_MODEL model)
/*******************************************************************************
* @brief	This function checks the existence of HSPPAD sensor and its model.
* @param	'model' indicates which HSPPAD series sensor the checked sensor is.
* 				The model actually indicates the PNUM of the sensor.
* 				Specially, HSPPAD_MODEL_NULL means the PNUM is not being checked.
* @retval	The model of the sensor, if it exists and fits the given model.
* 				Otherwise, HSPPAD_MODEL_NULL(0x00) will be returned.
*******************************************************************************/
{
	uint8_t a;
	
	DELAY_MS(5);//Wait for the sensor to start.
	if(!multi_bytes_read_flow(HSPPAD_WIA, &a, 1) || a!=HSPPAD_WIA_VALUE)//Check 'Who I Am'.
		return HSPPAD_MODEL_NULL;
	if(!multi_bytes_read_flow(HSPPAD_INFO, &a, 1) || a!=HSPPAD_INFO_VALUE)//Check INFO register.
		return HSPPAD_MODEL_NULL;
	if(!multi_bytes_read_flow(HSPPAD_PNUM, &a, 1))
		return HSPPAD_MODEL_NULL;
	if(model && a!=(uint8_t)model)//Check product number.
		return HSPPAD_MODEL_NULL;
	return (HSPPAD_MODEL)a;
}

static uint8_t self_test_cmd()
/*******************************************************************************
* @brief	This function makes HSPPAD sensor to run self-tests in command 
* 				action mode.
* @note		Access Check Test is the test of communication and register 
* 				accessibility.
* 				Self Test of Pressure Measurement is the test of GMR circuits and AD 
* 				converters.
* 				Note that there might be 'normal' pressure readings even if the sensor 
* 				is malfunctional in fact.
* @param	None
* @retval	The meaning of this byte is listed below:
* 				0x00	Tests passed, no error.
* 				0x01	The register value was abnormal before Access Check Test.
* 				0x02	The register value was abnormal during Access Check Test.
* 				0x03	The register value was abnormal after Access Check Test.
* 				0x04	Failed at the second step of Pressure Measurement Test.
* 				0x08	Failed at the third step of Pressure Measurement Test.
* 				0x10	The register value was abnormal before Pressure Measurement Test.
* 				0x20	The register value was abnormal after Pressure Measurement Test.
* 				0x55	Pressure Measurement Test did not start as expected.
* 				0xF0	Failed at the first step of Pressure Measurement Test.
* 				0xFF	Failed to communicate with sensor.
*******************************************************************************/
{
	uint8_t a;
	
	/*Access Check Test*/
	if(!multi_bytes_read_flow(HSPPAD_ACTR, &a, 1) || a!=HSPPAD_ST_DEF)
		return 0x01;
	if(!action_command_flow(HSPPAD_ACTC))
		return 0xFF;
	DELAY_MS(2);
	if(!multi_bytes_read_flow(HSPPAD_ACTR, &a, 1) || a!=HSPPAD_ST_OK)
		return 0x02;
	if(!multi_bytes_read_flow(HSPPAD_ACTR, &a, 1) || a!=HSPPAD_ST_DEF)
		return 0x03;
	
	/*Self Test of Pressure Measurement*/
	if(!multi_bytes_read_flow(HSPPAD_STR, &a, 1) || a!=HSPPAD_ST_DEF)
		return 0x10;
	if(!action_command_flow(HSPPAD_PSTC))
		return 0xFF;
	DELAY_MS(5);
	if(!multi_bytes_read_flow(HSPPAD_STR, &a, 1) || a!=HSPPAD_ST_OK)
		return a;
	if(!multi_bytes_read_flow(HSPPAD_STR, &a, 1) || a!=HSPPAD_ST_DEF)
		return 0x20;
	
	return 0x00;
}

uint8_t HSPPAD_Self_Test_Cmd()
/*******************************************************************************
* @brief	This function sets HSPPAD sensor to command action mode temporarily, 
* 				and makes the sensor to run self-tests.
* @note		There might be 'normal' pressure readings even if the sensor is 
* 				malfunctional in fact.
* 				Therefore, self-tests are indispensable for end users.
* @param	None
* @retval	The conclusion of self-tests. 0	means passed. Otherwise the value is 
* 				an errorcode.
*******************************************************************************/
{
	uint8_t a, b;
	
	/*Set to command mode.*/
	if(!multi_bytes_read_flow(HSPPAD_CTL2, &a, 1))
		return 0xFF;
	one_byte_write_flow(HSPPAD_CTL2, 0xA2);
	
	/*Run self-tests.*/
	b = self_test_cmd();
	
	/*Return to the former mode.*/
	one_byte_write_flow(HSPPAD_CTL2, a);
	
	return b;
}

uint8_t HSPPAD_Init(HSPPAD_Config_Struct *init_struct)
/*******************************************************************************
* @brief	This function sets the action mode of HSPPAD sensor.
* @note		The FIFO capacity is 16 bytes. Average is made with 16 pressure 
* 				data. Temperature is measured every 8th pressure measurement.
* @param	'init_struct' is the address of the user modified configuration struct.
* @retval	A boolean indicates if the control register writing flow is 
* 				successfully done. Value 1 means success, 0 means fail.
*******************************************************************************/
{
	uint8_t a;
	
	a = init_struct->filter_config;
	a &= HSPPAD_PTAP_MASK;//Deal with illegal values.
	a += HSPPAD_PDRP_MASK;
	if(!one_byte_write_flow(HSPPAD_CTL1, a))//Set the accuracy & power consumption.
		if(!one_byte_write_flow(HSPPAD_CTL1, a))
			return 0;
	if(init_struct->fifo_config)
	{
		if(!one_byte_write_flow(HSPPAD_FCTL, 0x90))//Enable FIFO.
			if(!one_byte_write_flow(HSPPAD_FCTL, 0x90))
				return 0;
	}
	else
	{
		if(!one_byte_write_flow(HSPPAD_FCTL, 0x10))//Disable FIFO.
			if(!one_byte_write_flow(HSPPAD_FCTL, 0x10))
				return 0;
	}
	if(init_struct->avg_config)
	{
		if(!one_byte_write_flow(HSPPAD_AVCL, 0x24))//Enable averaging.
			if(!one_byte_write_flow(HSPPAD_AVCL, 0x24))
				return 0;
	}
	else
	{
		if(!one_byte_write_flow(HSPPAD_AVCL, 0x38))//Disable averaging.
			if(!one_byte_write_flow(HSPPAD_AVCL, 0x38))
				return 0;
	}
	a = init_struct->mode;
	a &= HSPPAD_ODR_MODE_MASK;
	a += HSPPAD_TPMES_MASK;
	if(!one_byte_write_flow(HSPPAD_CTL2, a))//Set the measurement mode.
		if(!one_byte_write_flow(HSPPAD_CTL2, a))
			return 0;
	DELAY_MS(1);//Wait for mode switching.
	return 1;
}

uint8_t HSPPAD_Srst()
/*******************************************************************************
* @brief	This function makes HSPPAD sensor to execute a soft reset.
* @param	None
* @retval	A boolean indicates if the command is send successfully.
* 				Value 1 means success, 0 means fail.
*******************************************************************************/
{
	if(!one_byte_write_flow(HSPPAD_ACTL2, 0x80))//Set software resetting bit.
		if(!one_byte_write_flow(HSPPAD_ACTL2, 0x80))
			return 0;
	DELAY_MS(3);//Wait for reset completion.
	return 1;
}

uint8_t HSPPAD_Detection_Reg()
/*******************************************************************************
* @brief	This function writes specific registers of HSPPAD sensor to let it 
* 				measure temperature and pressure once.
* @note		The sensor should be set into register action mode previously.
* @param	None
* @retval	A boolean indicates if the command is send successfully.
* 				Value 1 means success, 0 means fail.
*******************************************************************************/
{
	if(!one_byte_write_flow(HSPPAD_ACTL1, 0x0A))
		return one_byte_write_flow(HSPPAD_ACTL1, 0x0A);
	return 1;
}

uint8_t HSPPAD_Detection_Cmd()
/*******************************************************************************
* @brief	This function commands HSPPAD sensor to measure temperature and 
* 				pressure once.
* @note 	The sensor should be set into command action mode previously.
* @param	None
* @retval	A boolean indicates if the command is send successfully.
* 				Value 1 means success, 0 means fail.
*******************************************************************************/
{
	if(!action_command_flow(HSPPAD_PTDET))
		return action_command_flow(HSPPAD_PTDET);
	return 1;
}

uint8_t HSPPAD_Check_Detection()
/*******************************************************************************
* @brief	This function checks if the measurement has finished.
* @note		It is recommended to give sufficient time for the measurement before 
* 				calling this function.
* 				Please refer to section 8.3 in the sensor manual to get the length of 
* 				this timespan.
* @param	None
* @retval	A boolean indicates if the mesurement is successfully done.
* 				Value 1 means success, 0 means error.
*******************************************************************************/
{
	uint8_t a, i = 137;
	
	do
	{
		if(multi_bytes_read_flow(HSPPAD_STAT, &a, 1) && !(a & HSPPAD_BUSY_MASK))
			return 1;
		DELAY_MS(2);
	}while(--i);
	return 0;
}

uint32_t HSPPAD_Get_Pressure_Pa(HSPPAD_MODEL model)
/*******************************************************************************
* @brief	This function gets pressure data from HSPPAD sensor.
* @param	'model' indicates which HSPPAD series sensor it is.
* @retval	The pressure data bytes are recorded in the 3 lower bytes of the 
* 				return value. The dimension is pascal.
* 				The highest byte is an errorcode. The meaning of it is listed below:
* 				0x00	No error.
* 				0x01	PRDY flag was not set. Old data were read.
* 				0x08	Failed to get pressure data.
*******************************************************************************/
{
	uint8_t a, b[3], i = 8;
	uint32_t prs;
	
	/*Check if there are new pressure data to read.*/
	do
	{
		if(multi_bytes_read_flow(HSPPAD_STAT, &a, 1) && a&HSPPAD_PRDY_MASK)
			break;
		DELAY_MS(1);
	}while(--i);
	
	/*Get pressure data.*/
	if(!multi_bytes_read_flow(HSPPAD_POUTL, b, 3))
		return 0x08000000;
	prs = b[0] + ((uint16_t)b[1] << 8) + ((uint32_t)b[2] << 16);
	
	/*Make a conversion to give the pressure data the correct dimension.*/
	switch(model)
	{
	case HSPPAD_MODEL_142A:
	case HSPPAD_MODEL_143A:
	case HSPPAD_MODEL_143C: prs <<= 1; break;
	case HSPPAD_MODEL_146A: prs *= 13; break;
	case HSPPAD_MODEL_147A: prs *= 6;
	}
	
	/*Add errorcode if PRDY==0 and the pressure data is not the latest.*/
	if(!i)
		prs += 0x01000000;
	
	return prs;
}

int16_t HSPPAD_Get_Temperature_Raw()
/*******************************************************************************
* @brief	This function gets temperature data from HSPPAD sensor.
* @param	None
* @retval	The function returns the temperature data when there is no error. The 
* 				dimension is 0.00390625 (=1/256) Celcius degree.
* 				Therefore, if the demension is Celcius degree, the higher byte is just 
* 				the integral part, and the lower byte is the decimal part.
* 				When the temperature value is beyond limit, that means the return 
* 				value is an errorcode. The meaning of it is listed below:
* 				-32768	TRDY flag was not set. No new datum to read out.
* 				-32576	Failed to get temperature data.
* 				other		Other problematic temperature reading.
*******************************************************************************/
{
	uint8_t a, b[2], i = 4;
	int16_t temp_raw;
	
	/*Check if there are new temperature data to read.*/
	do
	{
		if(multi_bytes_read_flow(HSPPAD_STAT, &a, 1) && a&HSPPAD_TRDY_MASK)
			break;
		DELAY_MS(1);
	}while(--i);
	if(!i)
		return -32768;
	
	/*Get temperature data.*/
	if(!multi_bytes_read_flow(HSPPAD_TOUTL, b, 2))
		return -32576;
	temp_raw = b[0] + ((uint16_t)b[1] << 8);
	
	return temp_raw;
}

float HSPPAD_Get_Temperature_CD()
/*******************************************************************************
* @brief	This function gets temperature data from HSPPAD sensor.
* @param	None
* @retval	The function returns the temperature data when there is no error. The 
* 				dimension is Celcius degree.
* 				When the temperature value is beyond limit, that means the return 
* 				value is an errorcode. The meaning of it is listed below:
* 				-128.0	Failed to get TRDY flag. No new datum to read out.
* 				-127.25	Failed to get temperature data.
* 				other		Other problematic temperature reading.
*******************************************************************************/
{
	return HSPPAD_Get_Temperature_Raw() * 0.00390625f;
}

uint8_t HSPPAD_Check_FIFO_Full()
/*******************************************************************************
* @brief	This function checks if the FIFO is full.
* @note		Use this only when FIFO is enabled.
* @param	None
* @retval	A boolean indicates if the FIFO is full.
* 				Value 1 means full, 0 means not (or communication failed).
*******************************************************************************/
{
	uint8_t a;
	
	if(!multi_bytes_read_flow(HSPPAD_FFST, &a, 1))
		if(!multi_bytes_read_flow(HSPPAD_FFST, &a, 1))
			return 0;
	return a & HSPPAD_FFEV_MASK;
}

uint8_t HSPPAD_Read_FIFO(uint32_t *dat, uint8_t num, HSPPAD_MODEL model)
/*******************************************************************************
* @brief	This function reads data from FIFO continually, until FIFO is empty, 
* 				or the data recording array is full.
* @note		Use this only when FIFO is enabled.
* @param	'dat' is the address of pressure data recording array.
* 				'num' is the maximum length of the array.
* 				'model' indicates which HSPPAD series sensor it is.
* @retval	The number of pressure data recorded by the array. It is never bigger 
* 				than the array length, or the number of data in FIFO.
*******************************************************************************/
{
	uint8_t a, i = 23, j = 0;
	
	do
	{
		if(!multi_bytes_read_flow(HSPPAD_FFST, &a, 1))
			continue;
		if(!(a & HSPPAD_FP_MASK))//Check if there is any datum in FIFO.
			break;
		*dat = HSPPAD_Get_Pressure_Pa(model);//Read data out.
		if(!((uint8_t)(*dat >> 24)))//Check validity.
		{
			dat++;
			j++;
		}
	}while(j<num && --i);
	return j;
}

/*Test Mode Functions*/
#if HSPPAD_TEST_MODE
#include "HSPPAD_Test_Mode.c"
#endif
