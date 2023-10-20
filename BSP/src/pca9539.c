/**
 * --------------------------------------------------------------------------
 * pca9555.c
 *
 *  Created on: Aug 16, 2020
 *      Author: Jack Lestrohan (c) Cobalt Audio - 2020
 * --------------------------------------------------------------------------
 */

#include "pca9539.h"
#include "myi2c.h"

/* max delay given to all I2C stuff to initialize itself */
#define I2C_READYNESS_DELAY		500

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

uint16_t i2c_key_value;
uint8_t key_update_flag;

PCA9555_HandleTypeDef PCA9555_LED=
{
	&gpio_init_struct,//&hi2c3,
	0X74,
	0,
	0,
	{0},
	{0},
	{0},
	0,
	0,
	0,
};

PCA9555_HandleTypeDef PCA9555_KEY=
{
	&gpio_init_struct,//&hi2c3,
	0X75,
	0,
	0,
	{0},
	{0},
	{0},
	0,
	0,
	0,
};


/**************************************************** 硬件IIC *********************************************************/

// /**
//  * Reads a given register
//  * @param hdev
//  * @param addr
//  * @return
//  */
// static HAL_StatusTypeDef pca9555_readRegister(PCA9555_HandleTypeDef *hdev, uint8_t addr, uint16_t *data) {
// 	// read the current GPINTEN
// 	HAL_StatusTypeDef ret;
// 	uint8_t datahigh = 0;
// 	uint8_t datalow = 0;
// 	uint16_t data_temp=0;

// 	/* reading the low byte */
// 	ret = HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr, 1, &datalow, 1, HAL_MAX_DELAY);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		ret = HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr, 1, &datalow, 1, HAL_MAX_DELAY);
// 		return ret;
// 	}
	

// 	/* reading the high byte */
// 	ret = HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr+1, 1, &datahigh, 1, HAL_MAX_DELAY);
// 	if (ret != HAL_OK)
// 	{
// 		// i2c3_reset();
// 		ret = HAL_I2C_Mem_Read(hdev->hi2c, hdev->addr, addr+1, 1, &datahigh, 1, HAL_MAX_DELAY);
// 		return ret;
// 	}
	

// 	/* builds the 16 bits value */
// 	data_temp=(datahigh << 8);
// 	*data = data_temp | datalow;

// 	return ret;
// }


// /**
//  * Writes a value to the given register
//  * @param hdev MCP23017_HandleTypeDef struct to the aimed interface
//  * @param regAddr Register Address
//  * @param regValue Value to write to
//  * @return
//  */
// static HAL_StatusTypeDef pca9555_writeRegister(PCA9555_HandleTypeDef *hdev, uint8_t regAddr, uint16_t regValue)
// {
// 	HAL_StatusTypeDef ret;
// 	uint8_t lowb = lowByte(regValue);
// 	uint8_t highb = highByte(regValue);

// 	ret = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, regAddr, 1, (uint8_t*) &lowb, 1, HAL_MAX_DELAY);
// 	if (ret != HAL_OK)
// 	{

// 		return ret;
// 	}
	
	

// 	ret = HAL_I2C_Mem_Write(hdev->hi2c, hdev->addr, regAddr+1, 1, (uint8_t*) &highb, 1, HAL_MAX_DELAY);
// 	if (ret != HAL_OK) 
// 	{
// 		return ret;
// 	}
	

// 	return HAL_OK;
// }

// /**
//  * Helper to update a single bit of an A/B register.
//  * 		Reads the current register value
//  * 		Writes the new register value
//  * @param hdev
//  * @param pin
//  * @param pValue
//  * @param portAaddr
//  * @param portBaddr
//  */
// static HAL_StatusTypeDef pca9555_updateRegisterBit(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_BitValue_t bitvalue, uint8_t commandByteAddr)
// {
// 	uint16_t regValue;
// 	HAL_StatusTypeDef ret;

// 	ret = pca9555_readRegister(hdev, commandByteAddr, &regValue);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		ret = pca9555_readRegister(hdev, commandByteAddr, &regValue);
// 		return ret;
// 	}
	

// 	// set the value for the particular bit
// 	bitWrite(regValue, pin, bitvalue);

// 	ret = pca9555_writeRegister(hdev, commandByteAddr, lowByte(regValue));
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		return ret;
// 	}
	

// 	ret = pca9555_writeRegister(hdev, commandByteAddr+1, highByte(regValue));
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		return ret;
// 	}

// 	return HAL_OK;
// }

// /**
//  * main initialization routine
//  * @param hdev
//  * @param hi2c
//  * @param addr
//  * @return
//  */
// HAL_StatusTypeDef pca9555_init(PCA9555_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr)
// {
// 	HAL_StatusTypeDef ret;

// 	hdev->hi2c = hi2c;
// 	hdev->addr = addr << 1;

// 	ret = HAL_I2C_IsDeviceReady(hi2c, hdev->addr, 20, I2C_READYNESS_DELAY);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		ret = HAL_I2C_IsDeviceReady(hi2c, hdev->addr, 20, I2C_READYNESS_DELAY);
// 		return ret;
// 	}	

// 	return HAL_OK;
// }

// /**
//  * Writes to a given pin
//  * @param hdev
//  * @param pin
//  * @param data
//  * @return
//  */
// HAL_StatusTypeDef pca9555_DigitalWrite(PCA9555_HandleTypeDef *hdev, uint8_t pin, GPIO_PinState pinState)
// {
// 	uint16_t data;
// 	HAL_StatusTypeDef ret;
	
// 	// read the current GPIO output latches
// 	ret = pca9555_readRegister(hdev, PCA9555_CB_OUTPUTS_PORTS, &data);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		ret = pca9555_readRegister(hdev, PCA9555_CB_OUTPUTS_PORTS, &data);
// 		return ret;
// 	}
	
	
// 	// set the pin and direction
// 	bitWrite(data, pin, pinState);
	
// 	pca9555_pinMode(&PCA9555_LED, pin, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);
// 	// write the new GPIO
	
// 	ret = pca9555_writeRegister(hdev, PCA9555_CB_OUTPUTS_PORTS, data);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		return ret;
// 	}
	
// 	return HAL_OK;
// }

// /**
//  * Reads a given pin
//  * @param hdev
//  * @param pin
//  * @return
//  */
// HAL_StatusTypeDef pca9555_digitalRead(PCA9555_HandleTypeDef *hdev, uint8_t pin, uint8_t *pinValue)
// {
// 	uint16_t regValue;
// 	HAL_StatusTypeDef ret; 
// 	hdev->key_out_pre = hdev->key_out;

// 	ret = pca9555_readRegister(hdev, PCA9555_CB_INPUTS_PORTS, &regValue);
// 	if (ret != HAL_OK) 
// 	{
// 		// i2c3_reset();
// 		ret = pca9555_readRegister(hdev, PCA9555_CB_INPUTS_PORTS, &regValue);
// 		if (ret != HAL_OK)
// 		{
// 			// i2c3_reset();
// 			ret = pca9555_readRegister(hdev, PCA9555_CB_INPUTS_PORTS, &regValue);
// 			if (ret != HAL_OK)
// 			{
// 				// i2c3_reset();
// 				ret = pca9555_readRegister(hdev, PCA9555_CB_INPUTS_PORTS, &regValue);
// 				return ret;
// 			}
// 		}
// 	}
	
// 	i2c_key_value = regValue;
// 	hdev->key_out = regValue;
// 	*pinValue = bitRead(regValue, pin);

// 	if (hdev->key_out != hdev->key_out_pre)
// 	{
// 		hdev->key_equal_flag = 1;
// 	}
// 	else
// 	{
// 		hdev->key_equal_flag = 0;
// 	}
	
// 	return HAL_OK;
// }


// /**
//  *
//  * @param hdev
//  * @param pin
//  * @param mode
//  * @param polarity
//  * @return
//  */
// HAL_StatusTypeDef ret1;
// HAL_StatusTypeDef pca9555_pinMode(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_PinMode_t mode, PCA9555_PinPolarity_t polarity)
// {
// 	HAL_StatusTypeDef ret;
// 	ret1=ret;
// 	/* first we set the right polarity */
// 	ret = pca9555_updateRegisterBit(hdev, pin, (PCA9555_BitValue_t)(polarity == PCA9555_POLARITY_INVERTED), PCA9555_CB_POL_INVERT_PORTS);
// 	if (ret != HAL_OK) return ret;

// 	/* let's setup the pin direction now */
// 	ret = pca9555_updateRegisterBit(hdev, pin, (PCA9555_BitValue_t)(mode == PCA9555_PIN_INPUT_MODE), PCA9555_CB_CONFIG_PORTS);
// 	if (ret != HAL_OK) return ret;

// 	return HAL_OK;
// }


/******************************************** 软件IIC ******************************************/

/**
 * @brief 写入输出端口寄存器
 * 
 * @param hdev 
 * @param cmd 
 * @param data 
 */
static void PCA9539_Write_Register(PCA9555_HandleTypeDef *hdev, uint8_t cmdByte, uint16_t data)
{
	uint8_t lowb = lowByte(data);
	uint8_t highb = highByte(data);

	iic_start();    /* 发送起始信号 */
	iic_send_byte(hdev->addr << 1);
	iic_wait_ack(); /* 每次发送完一个字节,都要等待ACK */
    iic_send_byte(cmdByte);
    iic_wait_ack();
	/* Data to Port */
	iic_send_byte(lowb);
	iic_wait_ack();
	iic_send_byte(highb);
	iic_wait_ack();
	iic_stop();		/* 产生一个停止条件 */
}

/**
 * @brief 读寄存器
 * 
 * @param hdev 
 * @param cmdByte 
 * @param data 
 * @return uint16_t 
 */
static uint16_t PCA9539_Read_Reg(PCA9555_HandleTypeDef *hdev, uint8_t cmdByte, uint16_t *data)
{
	uint8_t datahigh = 0;
	uint8_t datalow = 0;
	uint16_t data_temp = 0;

	iic_start();
	iic_send_byte(hdev->addr << 1);
	iic_wait_ack();             /* 每次发送完一个字节,都要等待ACK */
    iic_send_byte(cmdByte);
    iic_wait_ack();

	iic_start();
	iic_send_byte((hdev->addr << 1) + 1);
	iic_wait_ack();

	datalow = iic_read_byte(1);
	datahigh = iic_read_byte(0);
	iic_stop();

	data_temp = datahigh << 8;
	*data = data_temp + datalow;
	return *data;
}

/**
 * @brief 更新寄存器位的值
 * 
 * @param hdev 
 * @param pin 
 * @param bitvalue 
 * @param commandByteAddr 
 * @return HAL_StatusTypeDef 
 */
static HAL_StatusTypeDef PCA9539_updateRegisterBit(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_BitValue_t bitvalue, uint8_t commandByteAddr)
{
	uint16_t regValue;

	PCA9539_Read_Reg(hdev, commandByteAddr, &regValue);

	// set the value for the particular bit
	bitWrite(regValue, pin, bitvalue);

	PCA9539_Write_Register(hdev, commandByteAddr, lowByte(regValue));
	PCA9539_Write_Register(hdev, commandByteAddr, highByte(regValue));
	return HAL_OK;
}

/**
 * @brief 初始化
 * 
 */
void PCA9539_init(void)
{
	uint8_t status = 0;

	iic_init();
	if (IIC_READ_SDA && IIC_READ_SCL)
	{
		status = HAL_OK;
	}
	else
	{
		status = HAL_BUSY;
	}
	
	
	if (status == HAL_BUSY)
	{
		iic_init();
	}
}

/**
 * @brief 写指定引脚
 * 
 * @param hdev 
 * @param pin 
 * @param pinState 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef PCA9539_DigitalWrite(PCA9555_HandleTypeDef *hdev, uint8_t pin, GPIO_PinState pinState)
{
	uint16_t data = 0;
	PCA9539_Read_Reg(hdev, PCA9555_CB_OUTPUTS_PORTS, &data);

	// set the pin and direction
	bitWrite(data, pin, pinState);

	PCA9539_pinMode(&PCA9555_LED, pin, PCA9555_PIN_OUTPUT_MODE, PCA9555_POLARITY_NORMAL);

	PCA9539_Write_Register(hdev, PCA9555_CB_OUTPUTS_PORTS, data);
	return HAL_OK;
}

/**
 * @brief 读指定引脚
 * 
 * @param hdev 
 * @param pin 
 * @param pinValue 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef PCA9539_DigitalRead(PCA9555_HandleTypeDef *hdev, uint8_t pin, uint8_t *pinValue)
{
	uint16_t regValue;
	hdev->key_out_pre = hdev->key_out;

	PCA9539_Read_Reg(hdev, PCA9555_CB_INPUTS_PORTS, &regValue);

	i2c_key_value = regValue;
	hdev->key_out = regValue;
	*pinValue = bitRead(regValue, pin);

	if (hdev->key_out != hdev->key_out_pre)
	{
		hdev->key_equal_flag = 1;
	}
	else
	{
		hdev->key_equal_flag = 0;
	}
	
	return HAL_OK;
}

HAL_StatusTypeDef ret1;
/**
 * @brief 设置指定引脚的模式
 * 
 * @param hdev 
 * @param pin 
 * @param mode 
 * @param polarity 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef PCA9539_pinMode(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_PinMode_t mode, PCA9555_PinPolarity_t polarity)
{
	HAL_StatusTypeDef ret;
	ret1=ret;
	/* first we set the right polarity */
	ret = PCA9539_updateRegisterBit(hdev, pin, (PCA9555_BitValue_t)(polarity == PCA9555_POLARITY_INVERTED), PCA9555_CB_POL_INVERT_PORTS);
	if (ret != HAL_OK) return ret;

	/* let's setup the pin direction now */
	ret = PCA9539_updateRegisterBit(hdev, pin, (PCA9555_BitValue_t)(mode == PCA9555_PIN_INPUT_MODE), PCA9555_CB_CONFIG_PORTS);
	if (ret != HAL_OK) return ret;

	return HAL_OK;
}


