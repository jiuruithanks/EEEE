/**
 * @file status_monitor_tasks.c
 * @author Zhao Haofei
 * @brief ERCP״̬����������
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
 * @brief �����Բ�ѯ����
 * 
 * @param status    ״̬
 * @param has5048b  �Ŵ�����
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
 * @brief ��е�ĵ�λ�л�����
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
 * @brief ��е��״̬
 * 
 * @param status 
 * @param data 
 */
void ercp_device_feedback(STATUS_MONITOR *status, uint8_t *data)
{
    //data[7]:���
    if (ERCP_DEVICE.head_limit == 0)
    {
        //��
        data[7] |= (1 << 0);
    }
    else if (ERCP_DEVICE.back_limit == 0)
    {
        //��
        data[7] &= ~(1 << 0);
    }
    //data[7]��data[8]:��е״̬����λ����ϡ��˶����ͽ������  data[9]��ʵ���ٶ�
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

    //�жϽ����Ƿ�λ
    if (as5048b01.sum_length == 1100)
    {
        data[7] |= 1 << 1;
    }
    else
    {
        data[7] &= ~(1 << 1);
    }
    
     
    //��������
    data[8] = (data[8] & 0x00) | ((as5048b01.sum_length >> 4) & 0xFF);
    data[7] = (data[7] & 0x0F) | ((as5048b01.sum_length << 4) & 0xF0);
    // data[9] = data[9] & 0x00;
    data[9] = as5048b01.real_speed;

    //data[10]:��е����
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
 * @brief ��˿��״̬
 * 
 * @param status 
 * @param data 
 */
void ercp_gw_feedback(STATUS_MONITOR *status, uint8_t *data)
{
    //���
    if (ERCP_GW.head_limit == 0)
    {
        //��
        data[11] |= 1 << 0;
    }
    else if (ERCP_GW.back_limit == 0)
    {
        //��
        data[11] &= ~(1 << 0);
    }
    //data[11]��data[12]:��˿״̬����λ����ϡ��˶����ͽ������ data[13]:ʵ���ٶ�
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

     //�жϽ����Ƿ�λ
    if (as5048b00.sum_length == 1100)
    {
        data[11] |= 1 << 1;
    }
    else
    {
        data[11] &= ~(1 << 1);
    } 

    //��������
    data[12] = (data[12] & 0x0F) | ((as5048b01.sum_length >> 4) & 0xFF);
    data[11] = (data[11] & 0x0F) | ((as5048b01.sum_length << 4) & 0xF0);
    data[13] = as5048b00.real_speed;
}

/**
 * @brief ע����״̬
 * 
 * @param status 
 * @param data 
 * @param CS 
 */
void ercp_inject_feedback(STATUS_MONITOR *status, uint8_t *data, SPI_CS_TypeDef *CS)
{
    uint8_t v_set = 0;
    //��λѡ��
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

    //��е����
    if (status->inject_control_status < RECEIVE_DATA_GATE_L)
    {
        data[14] |= motor_head & 0x0F;
        //ת��
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
    //ѹ��
    data[16] = xgzp6877d_handle.pressure / 1000;
}


void Excute_Usart_TxData(void)
{
    check_TxData(UART2_Tx_Buf);
}



