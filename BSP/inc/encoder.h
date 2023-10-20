#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "bsp.h"
#include "crc.h"
#include "usart.h"
#include <string.h>


extern uint8_t Encoder_handler_buf[64];




uint8_t Read_protocol_encoder(void);
uint8_t Read_angle_injection(void);
void Check_RecData_CRC(uint32_t len);


#endif
