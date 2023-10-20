#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"
#include "tim.h"



void sys_delay_us(uint32_t nms); 
void tim_delay_us(uint16_t nus);
#endif
