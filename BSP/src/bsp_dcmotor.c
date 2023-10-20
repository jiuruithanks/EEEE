
#include "bsp_dcmotor.h"
#include "Tmc5160.h"
/*================================================================*/
/****
 * DCmotor1 导丝进退离合  包含上限位、下限位 
 * DCmotor2 器械进退离合  包含上限位、下限位 
 * 
 * 
 * 
 * ***/
/*================================================================*/

const uint8_t g_voltage_val = 12;

DCmotor_TypeDef DCmotor1={
                                  &htim1,
                                  TIM_CHANNEL_1,
                                  TIM_CHANNEL_2,
                                  head,
                                  DCMOTOR_STOP,
																	1,
																	1,
                                  LIMIT0_GPIO_Port,
                                  LIMIT0_Pin,
                                  LIMIT1_GPIO_Port,
                                  LIMIT1_Pin

                                  };
DCmotor_TypeDef DCmotor2={
                                  &htim1,
                                  TIM_CHANNEL_3,
                                  TIM_CHANNEL_4,
                                  head,
                                  DCMOTOR_STOP,
																	1,
																	1,
                                  LIMIT2_GPIO_Port,
                                  LIMIT2_Pin,
                                  LIMIT3_GPIO_Port,
                                  LIMIT3_Pin
                                  };
DCmotor_TypeDef DCmotor3={
																	&htim4,
																	TIM_CHANNEL_1,
																	TIM_CHANNEL_2,
																	head,
																	DCMOTOR_STOP,
																	1,
																	1,

                                };
DCmotor_TypeDef DCmotor4={
																	&htim4,
																	TIM_CHANNEL_3,
																	TIM_CHANNEL_4,
																	head,
																	DCMOTOR_STOP,
																	1,
																	1,
                                 
                              };

void Get_Limit_State(DCmotor_TypeDef *DCmotor)
{
  if (DCmotor->htimx == &htim1)
  {
    DCmotor->head_limit = HAL_GPIO_ReadPin(DCmotor->head_limit_GPIOx, DCmotor->head_limit_GPIO_Pin);
    DCmotor->back_limit = HAL_GPIO_ReadPin(DCmotor->back_limit_GPIOx, DCmotor->back_limit_GPIO_Pin);
  }
}

void Stop_DC_Motor(DCmotor_TypeDef *DCmotor )
{
  HAL_TIM_PWM_Stop(DCmotor->htimx, DCmotor->motor_p);
  HAL_TIM_PWM_Stop(DCmotor->htimx, DCmotor->motor_n);
}

void Run_DC_Motor(DCmotor_TypeDef *DCmotor, uint8_t dir, uint8_t speed_voltage)
{
  uint8_t  is_ok_to_run = ture;
  uint8_t  dir_temp = dir;
  uint8_t  speed_temp = speed_voltage;

  float    speed_pwm = 50 * speed_temp / 12 + 50;

  Get_Limit_State(DCmotor);
	
  if ((dir_temp == head) && (DCmotor->head_limit == 0))
  is_ok_to_run = false;
  else if ((dir_temp == back) && (DCmotor->back_limit == 0))
  is_ok_to_run = false;

  if (is_ok_to_run)
  {
    HAL_TIM_PWM_Start(DCmotor->htimx, DCmotor->motor_p);
    HAL_TIM_PWM_Start(DCmotor->htimx, DCmotor->motor_n);

    if(dir == head)
    {
    __HAL_TIM_SET_COMPARE(DCmotor->htimx, DCmotor->motor_p, speed_pwm);//设置第一路PWM脉冲宽度
    __HAL_TIM_SET_COMPARE(DCmotor->htimx, DCmotor->motor_n, 100 - speed_pwm);//设置第二路PWM脉冲宽度
    }
    else if(dir == back)
    {
    __HAL_TIM_SET_COMPARE(DCmotor->htimx, DCmotor->motor_p, 100 - speed_pwm);//设置第一路PWM脉冲宽度
    __HAL_TIM_SET_COMPARE(DCmotor->htimx, DCmotor->motor_n,speed_pwm);//设置第二路PWM脉冲宽度
    }
  }
  else
  {
    Stop_DC_Motor(DCmotor);
  }
}



