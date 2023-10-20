#ifndef __as5048b_H
#define __as5048b_H
#include "myi2c.h"
#include "usart.h"
#include "stdio.h"

#define READ_ANGLE_CMD 	0xFE
#define READ_ID			0X15
/* 左移一位后 */
#define ADDR_A			0x80 //0x40
#define ADDR_B			0x82 //0x41
#define ADDR_C			0x84 //0x42
#define ADDR_D			0x86 //0x43

typedef struct __AS5048B_HandleTypeDef {
	uint8_t 			ADDRESS;
//  	I2C_HandleTypeDef 	*HI2C;
 	uint8_t 			TxBuffer[1];
 	uint8_t 			RxBuffer[2];
	uint16_t 			angle_h;
	float  				angle_f;
	float  				angle_f_pre;
	
	uint8_t 	arrive_pos;							//判断导丝和器械是否到位
	uint8_t 	arrive_pos_pre;
	int32_t 	angle_one;							//不足一圈的角度值
	int32_t 	Cycle;								//圈数
	int32_t 	length;								//一圈内的长度
	int32_t 	sum_length;							//总长度
	uint32_t 	pos_head_move_ori;					//当前位置
	uint32_t 	pos_head_move_ori_pre;				//上一次的值
	int32_t 	pos_head_move_after_zero_process;	//累加值

	float 		angle_rpm;		
	float 		angle_rpm_pre;			
	float 		real_speed;			//实际转速

	uint8_t 	first_flag;
	
} AS5048B_HandleTypeDef;




extern AS5048B_HandleTypeDef as5048b00;
extern AS5048B_HandleTypeDef as5048b01;
extern AS5048B_HandleTypeDef as5048b10;
extern AS5048B_HandleTypeDef as5048b11;

void as5048b_init(AS5048B_HandleTypeDef *has5048b);
void AS5048B_Read(AS5048B_HandleTypeDef *has5048b, uint8_t addr, uint16_t *data);


// HAL_StatusTypeDef AS5048B_Read(AS5048B_HandleTypeDef *has5048b, uint8_t  addr, uint16_t *data) ;
// HAL_StatusTypeDef bsp_AS5048B_Init(AS5048B_HandleTypeDef* has5048b, uint8_t ADDRESS, I2C_HandleTypeDef* I2C_Handle);
#endif
