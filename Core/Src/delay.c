#include "delay.h"

/**
 * @brief 微秒级延时
 * 
 * @param nus 
 */
void sys_delay_us(uint32_t nus) 
{
    uint32_t temp; 
    SysTick->LOAD = 21 * nus; //外部时钟21M
    SysTick->VAL = 0X00;//清空计数器 
    SysTick->CTRL = 0X01;//使能，减到零是无动作，采用外部时钟源 
    do 
    { 
        temp = SysTick->CTRL;//读取当前倒计数值 
    }
    while((temp & 0x01) && (!(temp & (1<<16))));//等待时间到达 

    SysTick->CTRL = 0x00; //关闭计数器 
    SysTick->VAL = 0X00; //清空计数器 
}

/**
 * @brief 微秒级延时
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



