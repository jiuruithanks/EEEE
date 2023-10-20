#include "bsp_as5048b.h"

AS5048B_HandleTypeDef as5048b00 =
{
	.ADDRESS = ADDR_A
};
AS5048B_HandleTypeDef as5048b01 = 
{
	.ADDRESS = ADDR_B
};
AS5048B_HandleTypeDef as5048b10 = 
{
	.ADDRESS = ADDR_C
};
AS5048B_HandleTypeDef as5048b11 = 
{
	.ADDRESS = ADDR_D
};



void as5048b_init(AS5048B_HandleTypeDef *has5048b)
{
	has5048b->angle_f = 0.0;
	has5048b->angle_h = 0;
	has5048b->angle_one = 0;
	has5048b->Cycle = 0;
	has5048b->length = 0;
	has5048b->sum_length = 0;
	has5048b->pos_head_move_ori = 0;
	has5048b->pos_head_move_ori_pre = 0;
	has5048b->pos_head_move_after_zero_process = 0;
	has5048b->angle_rpm = 0;
	has5048b->angle_rpm_pre = 0;
	has5048b->real_speed = 0;
	has5048b->first_flag = 0;
	i2c_init();
}

/**
 * @brief 读取编码器的值
 * 
 * @param has5048b	编码器
 * @param addr 		设备地址
 * @param data 		存储的数据
 */
void AS5048B_Read(AS5048B_HandleTypeDef *has5048b, uint8_t addr, uint16_t *data)
{
	uint8_t datahigh = 0;
	uint8_t datalow = 0;

	i2c_start();
	i2c_send_byte(addr);
	i2c_wait_ack();
	i2c_send_byte(READ_ANGLE_CMD);
	i2c_wait_ack();

	i2c_start();
	i2c_send_byte(addr + 1);
	i2c_wait_ack();
	datahigh = i2c_read_byte(0);
	i2c_stop();
	

	i2c_start();
	i2c_send_byte(addr);
	i2c_wait_ack();
	i2c_send_byte(READ_ANGLE_CMD + 1);
	i2c_wait_ack();

	i2c_start();
	i2c_send_byte(addr + 1);
	i2c_wait_ack();
	datalow = i2c_read_byte(0);
	i2c_stop();

	*data = (datahigh << 6) | datalow;
	has5048b->angle_h = *data;
	has5048b->angle_f_pre = has5048b->angle_f;
	has5048b->angle_f = has5048b->angle_h * 360.0 / 16383;
}



// HAL_StatusTypeDef bsp_AS5048B_Init(AS5048B_HandleTypeDef* has5048b, uint8_t ADDRESS, I2C_HandleTypeDef* I2C_Handle)
// {

// 	HAL_StatusTypeDef ret;

// 	has5048b->HI2C = I2C_Handle;
// 	has5048b->ADDRESS = ADDRESS << 1;
// 	has5048b->angle_f=0.0;
// 	has5048b->angle_h=0;
// 	has5048b->angle_one = 0;
// 	has5048b->Cycle = 0;
// 	has5048b->length = 0;
// 	has5048b->sum_length = 0;
// 	has5048b->pos_head_move_ori = 0;
// 	has5048b->pos_head_move_ori_pre = 0;
// 	has5048b->pos_head_move_after_zero_process = 0;
// 	has5048b->angle_rpm = 0;
// 	has5048b->angle_rpm_pre = 0;
// 	has5048b->real_speed = 0;
// 	has5048b->first_flag = 0;

	// ret = HAL_I2C_IsDeviceReady(has5048b->HI2C, has5048b->ADDRESS, 20, 10);
// 	if (ret != HAL_OK) return ret;

// 	return HAL_OK;
// }




// HAL_StatusTypeDef ret_test;
// HAL_StatusTypeDef AS5048B_Read(AS5048B_HandleTypeDef *has5048b, uint8_t addr, uint16_t *data) 
// {	HAL_StatusTypeDef ret;
// 	// read the current GPINTEN

// 	uint8_t datahigh = 0;
// 	uint8_t datalow = 0;
// 	uint16_t data_temp=0;

// 	ret_test = HAL_I2C_IsDeviceReady(has5048b->HI2C, has5048b->ADDRESS, 20, 1);
// 	if (ret_test != HAL_OK) 
// 	return ret_test;
// 	/* reading the low byte */
//	ret_test = HAL_I2C_Mem_Read(has5048b->HI2C, has5048b->ADDRESS, addr, 1, &datahigh, 1, 1);
// 	if (ret_test != HAL_OK) 
		
// 	return ret_test;

// 	/* reading the high byte */
	// ret_test = HAL_I2C_Mem_Read(has5048b->HI2C, has5048b->ADDRESS, addr+1, 1, &datalow, 1, 1);
// 	if (ret_test != HAL_OK) return ret;

// 	/* builds the 16 bits value */
	// data_temp = (datahigh << 6);
	// *data = data_temp | datalow;
	// has5048b->angle_h = data_temp | datalow;
	// has5048b->angle_f_pre = has5048b->angle_f;
	// has5048b->angle_f = has5048b->angle_h * 360.0 / 16383;

// 	return ret_test;
// }





