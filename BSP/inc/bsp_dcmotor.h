#ifndef __BSP_DCMOTOR_H
#define __BSP_DCMOTOR_H
#include "main.h"
#include "tim.h"


typedef enum
{
	DCMOTOR_STOP = 0x00,//Ultra Low Power
	DCMOTOR_RUN  = 0x01//Low Power

}DCMotor_status;

typedef enum
{
	head=0x00,
	back=0x01

}DCMOTOR_DIR;

typedef struct {
  TIM_HandleTypeDef * htimx; 
  uint32_t           motor_p;
  uint32_t           motor_n;
  DCMOTOR_DIR        dir;
  DCMotor_status     status;

	uint8_t           head_limit;
  uint8_t           back_limit;
  GPIO_TypeDef*     head_limit_GPIOx;
  uint16_t          head_limit_GPIO_Pin;
  GPIO_TypeDef*     back_limit_GPIOx;
  uint16_t          back_limit_GPIO_Pin;
} DCmotor_TypeDef;


extern DCmotor_TypeDef DCmotor1;
extern DCmotor_TypeDef DCmotor2;
extern DCmotor_TypeDef DCmotor3;
extern DCmotor_TypeDef DCmotor4;

extern const uint8_t g_voltage_val;



void Get_Limit_State(DCmotor_TypeDef *DCmotor);
void Stop_DC_Motor(DCmotor_TypeDef *DCmotor );
void Run_DC_Motor(DCmotor_TypeDef *DCmotor, uint8_t dir, uint8_t speed_voltage);


#endif
