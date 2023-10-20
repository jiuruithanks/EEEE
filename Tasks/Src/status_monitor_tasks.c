/**
 * @file status_monitor_tasks.c
 * @author Zhao Haofei
 * @brief ERCP状态监测相关任务
 * @version 1.0
 * @date 2023-08-3
 *
 * @copyright Copyright (c) 2023
 */
#include "status_monitor_tasks.h"
#include "pca9539.h"
#include "bsp.h"
#include "bsp_dcmotor.h"
#include <math.h>
#include "XGZP6877D.h"

uint8_t g_usart_flag;

STATUS_MONITOR status_monitor = 
{
    0,
    0,
    0x7F,
    0x7F,
    0x7F,
    0x7F,
    0x7F,
    0x7F
};

/**
 * @brief 周期性查询数据
 * 
 * @param status    状态
 * @param has5048b  磁传感器
 */
void data_collection(STATUS_MONITOR *status)
{
    status->encoder 				= PCA9555_KEY.rotate;
    status->gw_status 				= delete_esc_data[7];
    status->device_status 			= delete_esc_data[5];
    status->gw_device_status 		= delete_esc_data[9];
	status->dev_control_status		= delete_esc_data[6];
	status->inject_select_status 	= delete_esc_data[11];
    status->inject_control_status 	= delete_esc_data[8];	
}

void mode_select(uint8_t mode)
{
    if (mode)
    {
        UART2_Tx_Buf[5] = 1;
        UART2_Tx_Buf[6] = 0;
        UART2_Tx_Buf[7] = 0;
        UART2_Tx_Buf[8] = 0;
        UART2_Tx_Buf[9] = 0;
        UART2_Tx_Buf[10] = 0;
        UART2_Tx_Buf[11] = 0;
        UART2_Tx_Buf[12] = 0;
        UART2_Tx_Buf[13] = 0;
    }
    else
    {
        UART2_Tx_Buf[5] = 0;
        UART2_Tx_Buf[14] = 0;
        UART2_Tx_Buf[15] = 0;
        UART2_Tx_Buf[16] = 0;
    }
}

/**
 * @brief 器械的档位切换反馈
 * 
 * @param status 
 * @param data 
 */
void ercp_switch_feedback(STATUS_MONITOR *status, uint8_t *data)
{
    
    if ((status->encoder == 0x08))
    {
        data[6] = cutter;
    }
    else if ((status->encoder == 0x07))
    {
        data[6] = sacculus;
    }
    else if ((status->encoder == 0x06))
    {
        data[6] = net_basket;
    }
    else if ((status->encoder == 0x05))
    {
        data[6] = drainage_tube;
    }
    else
    {
        data[6] = biliary_stent;
    }        
}

/**
 * @brief 器械的状态
 * 
 * @param status 
 * @param data 
 */
void ercp_device_feedback(STATUS_MONITOR *status, uint8_t *data)
{
    //data[7]:离合
    if (ERCP_DEVICE.head_limit == 0)
    {
        //合
        data[7] |= (1 << 0);
    }
    else if (ERCP_DEVICE.back_limit == 0)
    {
        //开
        data[7] &= ~(1 << 0);
    }
    //data[7]、data[8]:器械状态（到位、离合、运动）和进入深度  data[9]：实际速度
    if ((status->device_status) < RECEIVE_DATA_GATE_L)
    {
        data[7] &= ~0x0C;
        data[7] |= (motor_head << 2) & 0x0C;    
        
    }
    else if ((status->device_status) > RECEIVE_DATA_GATE_H)
    {
        data[7] &= ~0x0C;
        data[7] |= (motor_back << 2) & 0x0C;
    }
    else
    {
        data[7] &= ~0x0C;
        data[7] |= (motor_stop << 2) & 0x0C;
    }

    //判断进入是否到位
    if (as5048b01.sum_length == 1100)
    {
        data[7] |= 1 << 1;
    }
    else
    {
        data[7] &= ~(1 << 1);
    }
    
     
    //进入的深度
    data[8] = (data[8] & 0x00) | ((as5048b01.sum_length >> 4) & 0xFF);
    data[7] = (data[7] & 0x0F) | ((as5048b01.sum_length << 4) & 0xF0);
    // data[9] = data[9] & 0x00;
    data[9] = as5048b01.real_speed;

    //data[10]:器械控制
    if (status->dev_control_status < RECEIVE_DATA_GATE_L)
    {
        data[10] = motor_head;
    }
    else if (status->dev_control_status > RECEIVE_DATA_GATE_H)
    {
        data[10] = motor_back;
    }
    else
    {
        data[10] = motor_stop;
    }
}

/**
 * @brief 导丝的状态
 * 
 * @param status 
 * @param data 
 */
void ercp_gw_feedback(STATUS_MONITOR *status, uint8_t *data)
{
    //离合
    if (ERCP_GW.head_limit == 0)
    {
        //合
        data[11] |= 1 << 0;
    }
    else if (ERCP_GW.back_limit == 0)
    {
        //开
        data[11] &= ~(1 << 0);
    }
    //data[11]、data[12]:导丝状态（到位、离合、运动）和进入深度 data[13]:实际速度
    if (status->gw_status < RECEIVE_DATA_GATE_L)
    {
        data[11] &= ~0x0C;
        data[11] |= (motor_head << 2) & 0x0C;
       
    }
    else if (status->gw_status > RECEIVE_DATA_GATE_H)
    {
        data[11] &= ~0x0C;
        data[11] |= (motor_back << 2) & 0x0C;
    }
    else
    {
        data[11] &= ~0x0C;
        data[11] |= (motor_stop << 2) & 0x0C;
    }

     //判断进入是否到位
    if (as5048b00.sum_length == 1100)
    {
        data[11] |= 1 << 1;
    }
    else
    {
        data[11] &= ~(1 << 1);
    } 

    //进入的深度
    data[12] = (data[12] & 0x0F) | ((as5048b01.sum_length >> 4) & 0xFF);
    data[11] = (data[11] & 0x0F) | ((as5048b01.sum_length << 4) & 0xF0);
    data[13] = as5048b00.real_speed;
}

/**
 * @brief 注射库的状态
 * 
 * @param status 
 * @param data 
 * @param CS 
 */
void ercp_inject_feedback(STATUS_MONITOR *status, uint8_t *data, SPI_CS_TypeDef *CS)
{
    uint8_t v_set = 0;
    //档位选择
    if ((status->inject_select_status) == 1)
    {
        // data[14] &= ~0xF0;
        data[14] = (attract << 4) & 0xF0;
    }
    else if ((status->inject_select_status) == 2)
    {
        // data[14] &= ~0xF0;
        data[14] = (water << 4) & 0xF0;
    }
    else if ((status->inject_select_status) == 3)
    {
        // data[14] &= ~0xF0;
        data[14] = (radiography << 4) & 0xF0;
    }
    else if ((status->inject_select_status) == 4)
    {
        // data[14] &= ~0xF0;
        data[14] = (air << 4) & 0xF0;
    }

    v_set = (CS->vmax / (200 * 256 * 1.4)) * 60;
    if (v_set > 255)
    {
        v_set = 255;
    }

    //器械控制
    if (status->inject_control_status < RECEIVE_DATA_GATE_L)
    {
        data[14] |= motor_head & 0x0F;
        //转速
        data[15] = v_set * (1 - (float)status->inject_control_status / 98);
    }
    else if (status->inject_control_status > RECEIVE_DATA_GATE_H)
    {
        data[14] |= motor_back & 0x0F;
        data[15] = v_set * ((float)status->inject_control_status / 256);
    }
    else
    {
        data[14] |= motor_stop & 0x0F;
        data[15] = 0;
    }   
    //压力
    data[16] = xgzp6877d_handle.pressure / 1000;
}


void Excute_Usart_TxData(void)
{
    check_TxData(UART2_Tx_Buf);
}



