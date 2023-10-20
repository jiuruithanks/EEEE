#include "delay.h"

/**
 * @brief ΢�뼶��ʱ
 * 
 * @param nus 
 */
void sys_delay_us(uint32_t nus) 
{
    uint32_t temp; 
    SysTick->LOAD = 21 * nus; //�ⲿʱ��21M
    SysTick->VAL = 0X00;//��ռ����� 
    SysTick->CTRL = 0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ 
    do 
    { 
        temp = SysTick->CTRL;//��ȡ��ǰ������ֵ 
    }
    while((temp & 0x01) && (!(temp & (1<<16))));//�ȴ�ʱ�䵽�� 

    SysTick->CTRL = 0x00; //�رռ����� 
    SysTick->VAL = 0X00; //��ռ����� 
}

/**
 * @brief ΢�뼶��ʱ
 * 
 * @param nus 
 */
void tim_delay_us(uint16_t nus)
{
	__HAL_TIM_SetCounter(&htim7, 0);
	__HAL_TIM_ENABLE(&htim7);
	
	while(__HAL_TIM_GetCounter(&htim7) < nus);
	__HAL_TIM_DISABLE(&htim7);
}



