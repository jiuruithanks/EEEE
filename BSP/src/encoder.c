#include "encoder.h"
#include "usart.h"


extern uint8_t UART_addr;

/*
****************************************************
*	函 数 名: Read_angle_injection
*	功能说明: 读取注射库控制（角度传感器4）的角度
*   形    参: 无
*	返 回 值: 旋钮的角度
****************************************************
*/
uint8_t Read_angle_injection(void)
{
    return Encoder_handler_buf[8];
}

/*
****************************************************
*	函 数 名: Read_protocol_encoder
*	功能说明: 读取编码器的档位
*   形    参: 无
*	返 回 值: 编码器的档位
****************************************************
*/
uint8_t Read_protocol_encoder(void)
{
//    if (Encoder_Rx_Flag = 1)
//    {
        if (Encoder_handler_buf[11] == 0x01)
        {  
        return 1; 
        }
        else if (Encoder_handler_buf[11] == 0x02)
        {
            return 2;
        }
        else if (Encoder_handler_buf[11] == 0x03)
        {
            return 3;
        }
        else if (Encoder_handler_buf[11] == 0x04)
        {
            return 4;
        }
        else
        {
            return 0;
        }
//    }
//    else
//    {
//         Encoder_handler_buf[11] = 0xFF;
//         return 0XFF;
//    }
}



uint8_t Encoder_handler_buf[64];
/*
****************************************************
*	函 数 名: Check_RecData_CRC
*	功能说明: 校验一包数据
*   形    参: len:数据长度
*	返 回 值: 无
****************************************************
*/
void Check_RecData_CRC(uint32_t len)
{
	Uint16ToByte_Typedef Uint16ToByte_CRC16 = {0};
 //   clean_data();
	delete_esc(delete_esc_data ,UART1_Rx_Buf_reg ,&len);
	
	if( delete_esc_data[0] == UART_addr) //确定第一位是不是0X02
	{
		if( len == delete_esc_data[1] + 2 ) //包长度+2个字节(CRC)
		{
            if (delete_esc_data[2] == 0x11) //操控平台扩展板的发出者是不是0x11
            {
                if (delete_esc_data[3] == 0x01) //操控平台扩展板的接收者是不是0x11
                {
                    if (delete_esc_data[4] == 0x03) //操控平台扩展板的包类型是不是0x03
                    {
                        Uint16ToByte_CRC16.data[0] = delete_esc_data[ delete_esc_data[1] ];     //CRC低8位 
			            Uint16ToByte_CRC16.data[1] = delete_esc_data[ delete_esc_data[1] + 1 ]; //CRC高8位
			            if((Uint16ToByte_CRC16.data_uint16_t == CRC16_Modbus(delete_esc_data, delete_esc_data[1])) && (CRC16_Modbus(delete_esc_data, delete_esc_data[1]) != 0))
                        {
                            memcpy(Encoder_handler_buf, delete_esc_data, sizeof(delete_esc_data));

                            clean_data();
                        }
                    }
                    else 
                    {
                        clean_data();
                    } 
                }  
                else 
                {
                    clean_data();
                }    
            } 
            else 
            {
                clean_data();
            }   
        }
        else 
        {
            clean_data();
        } 	   
	}
    else 
    {
        clean_data();
    }
}
