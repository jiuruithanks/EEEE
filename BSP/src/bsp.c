#include <math.h>
#include "bsp.h"
#include "led_key_tasks.h"
#include "motor_tasks.h"
#include "myi2c.h"
#include "task.h"
#include "tmc5160.h"
#include "modbus_crc16.h"
#include "XGZP6877D.h"
#include "status_monitor_tasks.h"
#include "sensor_tasks.h"

uint8_t bsp_RunPer10ms_flag=0;
uint8_t bsp_RunPer50ms_flag;
uint8_t bsp_RunPer100ms_flag=0;
uint8_t bsp_RunPer200ms_flag;
uint8_t bsp_RunPer500ms_flag=0;

uint8_t  pre_pre_state;
uint8_t  pre_state;
uint8_t  current_state;
uint8_t	 manual_head_speed;
uint8_t	 manual_back_speed;

RS485_KEY_T rs485_key,rs485_key_pre;
RS485_KEY_ACTION_T  rs485_action;

DEVICE_GW_Typedef device_move, device_action, gw_move, device_and_gw_move, inject_action, encoder_select;


unsigned char manual_head_data[10];
unsigned char manual_back_data[10];
unsigned char manual_stop_data[10];
 
uint16_t Crc_data_cal_h;
uint16_t Crc_data_cal_b;
uint16_t Crc_data_cal_s;

uint8_t inspire_motor_flag=0; 
uint8_t state_change_flag=0;

uint8_t auto_manual_control_flag=0;      ///自动、手动模式标志位
uint8_t manual_stop_motor_flag;   ///初次时，停止所有电机  再循环时，为1 不再停止 电机
uint8_t auto_stop_motor_flag;

uint8_t Uartmotor3_to_max[9];
uint8_t Uartmotor3_to_min[9];
uint8_t Uartmotor3_target[9];		
uint8_t Uartmotor3_position[9];	
		

uint16_t angle_as5048b_h;
uint8_t  bit_value;

int32_t  press_for_test;
uint32_t speed_for_test;

uint8_t  UART_addr = 2;




void init_rs485_key(void)
{
	RS485_KEY_T rs485_key={0};
	RS485_KEY_T	rs485_key_pre={0};
}

uint8_t Get_Mode(void)
{
    uint8_t mode;
    mode = HAL_GPIO_ReadPin(CPU_MODE0_GPIO_Port, CPU_MODE0_Pin) | (HAL_GPIO_ReadPin(CPU_MODE1_GPIO_Port, CPU_MODE1_Pin) << 1);
	return mode;
}

void update_flag(void)
{	
	rs485_key_pre.up_flag			=rs485_key.up_flag;	//
	rs485_key_pre.down_flag			=rs485_key.down_flag;		//
	rs485_key_pre.right_flag		=rs485_key.right_flag;		//
	rs485_key_pre.left_flag 		= rs485_key.left_flag;			//

	rs485_key_pre.triangle_flag		=rs485_key.triangle_flag;				//
	rs485_key_pre.circle_flag		=rs485_key.circle_flag;			//
	rs485_key_pre.cross_flag		=rs485_key.cross_flag;				//
	rs485_key_pre.square_flag		=rs485_key.square_flag;			//

	rs485_key_pre.L1_flag			=rs485_key.L1_flag;				//
	rs485_key_pre.L2_flag			=rs485_key.L2_flag;			//
	rs485_key_pre.L3_flag			=rs485_key.L3_flag;				//
	rs485_key_pre.R1_flag			=rs485_key.R1_flag;			//

	rs485_key_pre.R2_flag			=rs485_key.R2_flag ;				//
	rs485_key_pre.R3_flag			=rs485_key.R3_flag;			//
	rs485_key_pre.SHARE_flag		=rs485_key.SHARE_flag;				//
	rs485_key_pre.OPTIONS_flag		=rs485_key.OPTIONS_flag;			//

	rs485_key_pre.L3_L_flag			=rs485_key.L3_L_flag;				//
	rs485_key_pre.L3_R_flag			=rs485_key.L3_R_flag;			//
	rs485_key_pre.L3_UP_flag		=rs485_key.L3_UP_flag;				//
	rs485_key_pre.L3_DOWN_flag		=rs485_key.L3_DOWN_flag;			//

	rs485_key_pre.R3_L_flag			=rs485_key.R3_L_flag;				//
	rs485_key_pre.R3_R_flag			=rs485_key.R3_R_flag;			//
	rs485_key_pre.R3_UP_flag		=rs485_key.R3_UP_flag;				//
	rs485_key_pre.R3_DOWN_flag		=rs485_key.R3_DOWN_flag;			

	rs485_key_pre.dec_head_flag		=rs485_key.dec_head_flag;
	rs485_key_pre.dec_back_flag		=rs485_key.dec_back_flag;
	rs485_key_pre.dec_left_flag		=rs485_key.dec_left_flag;
	rs485_key_pre.dec_right_flag 	=rs485_key.dec_right_flag;

	rs485_key_pre.backup0_flag		=rs485_key.backup0_flag;				//
	rs485_key_pre.backup1_flag		=rs485_key.backup1_flag;			//
	rs485_key_pre.backup2_flag		=rs485_key.backup2_flag;				//
	rs485_key_pre.backup3_flag		=rs485_key.backup3_flag;			//

	rs485_key_pre.backup4_flag		=rs485_key.backup4_flag;				//
	rs485_key_pre.backup5_flag		=rs485_key.backup5_flag;			//
	rs485_key_pre.backup6_flag		=rs485_key.backup6_flag;				//
	rs485_key_pre.backup7_flag		=rs485_key.backup7_flag;			//
}

void Action_Detect(RS485_KEY_T *key, RS485_KEY_T *key_pre, RS485_KEY_ACTION_T *action)
{
	uint8_t* p_pre = (uint8_t*)key;
	uint8_t* p = (uint8_t*)key_pre;	
	uint8_t* p_t = (uint8_t*)action;
	uint8_t i;
	for(i = 0; i < sizeof(RS485_KEY_T) / sizeof(uint8_t); i++)
	{
		if((*(p_pre++) == 0) && ((*p++) == 1) )
		(*p_t++)++;

		if(*p_t > 2)
		*p_t = 0;
	}
} 

void update_flag_typedef(void)
{
	uint8_t * p_pre = (uint8_t*)&rs485_key_pre;
	uint8_t * p 	= (uint8_t*)&rs485_key;
	uint8_t i;
	
	for(i = 0; i < sizeof(RS485_KEY_T) / sizeof(uint8_t); i++)
	{
		*(p_pre++) = (*p++);
	}
}

/**
 * @brief The data after escaping becomes the data before escaping
 * @param data		Data before escaping
 * @param data_esc	The escaped data
 * @param length	Number of valid data
 * @retval
*/
void delete_esc(unsigned char *data, unsigned char *data_esc, unsigned int *length)
{
	unsigned char i = 0, j = 0;
	for(i = 0, j = 0; i < *length; i++, j++)
	{
		if(data_esc[i] == 0x1b )
		{
			if( data_esc[i + 1] == 0x00 )
			{
				data[j] = 0x1b;
			}
			else if( data_esc[i + 1] == 0xe7 )
			{
				data[j] = 0x02;
			}
			i++;
		}
		else
		{
			data[j] = data_esc[i];
		}
	}
	*length = j;
}

/**
 * @brief Data converted from before to after escaping
 * @param data		Data before escaping
 * @param data_esc	The escaped data
 * @param length	Number of valid data
 * @retval
*/
void inset_esc(unsigned char *data, unsigned char *length)
{
	unsigned char buf[64];
	unsigned char i = 0, j = 0;
	
	memcpy(buf, data, *length);
	
	for(i = 1, j = 1; i < *length; i++, j++)
	{
		if( buf[i] == 0x02 )
		{
			data[j] = 0x1b;
			data[++j] = 0xe7;
		}
		else if(buf[i] == 0x1b)
		{
			data[j] = 0x1b;
			data[++j] = 0x00;
		}
		else
		{
			data[j] = buf[i];
		}
	}
	
	*length = j;
}

/// @brief ///ERCP  used to receive message from extend board of platform 
/// to contorl device control /gw fwd /bwd forceps lifting .
/// @param delete_esc_data 
void getkey(unsigned char *delete_esc_data)
{
	unsigned char *p 			= delete_esc_data;
	device_move.rec_data 		= p[5];				//器械进退
	device_action.rec_data 		= p[6];				//器械控制
	gw_move.rec_data 			= p[7];				//导丝进退
	device_and_gw_move.rec_data = p[9];				//导丝和器械组合进退
	
	inject_action.rec_data 		= p[8];				//注射库控制
	encoder_select.rec_data 	= p[11];			//注射库选择

//	device_move.move_head_uartflag=p[9];
//	device_move.move_back_uartflag=p[10];
}

/// @brief 更新步进电机的参数(旋钮相关)
/// @param device_actionx 			control uart motor action 	切割刀的串口电机
/// @param gw_movex       			contorl gw action 			导丝进退
/// @param device_movex   			contorl device action		器械进退
/// @param device_and_gw_movex   	contorl device and gw action导丝和器械的组合进退
void update_rec_motor_para(DEVICE_GW_Typedef *gw_movex, DEVICE_GW_Typedef *device_movex, DEVICE_GW_Typedef *device_and_gw_movex, DEVICE_GW_Typedef *device_actionx)
{
	//gw action导丝进退
	if (gw_movex->rec_data < RECEIVE_DATA_GATE_L)
	{
		gw_movex->run_dir = 0;
		gw_movex->run_uartflag = 1;
		gw_movex->run_speed = abs(127 - gw_movex->rec_data) * 100 / 127;
	}
	else if (gw_movex->rec_data > RECEIVE_DATA_GATE_H)
	{
		gw_movex->run_dir = 1;
		gw_movex->run_uartflag = 1;
		gw_movex->run_speed = abs(gw_movex->rec_data - 128) * 100 / 127;
	}
	else 
	{
		gw_movex->run_uartflag = 0;
		gw_movex->run_speed = 0;
	}

	//device action器械进退
	if (device_movex->rec_data < RECEIVE_DATA_GATE_L)
	{
		device_movex->run_dir = 0;
		device_movex->run_uartflag = 1;
		device_movex->run_speed = abs(127 - device_movex->rec_data) * 100 / 127;

	}
	else if (device_movex->rec_data > RECEIVE_DATA_GATE_H)
	{
		device_movex->run_dir = 1;
		device_movex->run_uartflag = 1;
		device_movex->run_speed = abs(device_movex->rec_data - 128) * 100 / 127;
	}
	else 
	{
		device_movex->run_uartflag = 0;
		device_movex->run_speed = 0;
	}

	//gw and device combined action 导丝和器械的组合进退
	if (device_and_gw_movex->rec_data < RECEIVE_DATA_GATE_L)
	{
		/* 导丝进，器械退 */
		device_and_gw_movex->gw_dev_flag = 1;
		device_and_gw_movex->dev_gw_flag = 0;
		device_and_gw_movex->run_speed = fabs(127 - device_and_gw_movex->rec_data) * 100 / 127;	
	}
	else if (device_and_gw_movex->rec_data > RECEIVE_DATA_GATE_H)
	{
		/* 导丝退，器械进 */
		device_and_gw_movex->gw_dev_flag = 0;
		device_and_gw_movex->dev_gw_flag = 1;
		device_and_gw_movex->run_speed = fabs(device_and_gw_movex->rec_data - 128) * 100 / 127;
	}
	else 
	{
		/* 不动作 */
		device_and_gw_movex->gw_dev_flag = 0;
		device_and_gw_movex->dev_gw_flag = 0;
	}

	//uart motor action串口电机
	if (device_actionx->rec_data < RECEIVE_DATA_GATE_L)
	{
		device_actionx->run_dir = 0;
		device_actionx->run_uartflag = 1;
		device_actionx->run_speed = abs(127 - device_actionx->rec_data) * 100 / 127;
	}
	else if (device_actionx->rec_data > RECEIVE_DATA_GATE_H)
	{
		device_actionx->run_dir = 1;
		device_actionx->run_uartflag = 1;
		device_actionx->run_speed = abs(device_actionx->rec_data - 128) * 100 / 127;
	}
	else 
	{
		device_actionx->run_uartflag = 0;
		device_actionx->run_speed = 0;
	}
}

/**
 * @brief 跟新注射库控制的标志位
 * 
 * @param inject_action 
 */
void update_inject_symbol(DEVICE_GW_Typedef *inject_action)
{
	//注射库相关操作
	if (inject_action->rec_data < RECEIVE_DATA_GATE_L)
	{
		inject_action->run_dir = 0;
		inject_action->run_speed = abs(127 - inject_action->rec_data) * 100 / 127;
	}
	else if (inject_action->rec_data > RECEIVE_DATA_GATE_H)
	{
		inject_action->run_dir = 1;
		inject_action->run_speed = abs(inject_action->rec_data - 128) * 100 / 127;
	}
	else
	{
		inject_action->run_dir = 2;
		inject_action->run_speed = 0;
	}
}

/**
 * @brief 改变ERCP的目标角度
 * 
 * @param tar_pos 0, 1, 2, 3
 * @return uint16_t 
 */
uint16_t change_target_angle(uint8_t tar_pos)	
{
	uint8_t tar_pos_temp = tar_pos;
	uint16_t tar_angle_temp = 0;

	switch(tar_pos_temp)
	{
		case 0: tar_angle_temp = 0; break;     //	122
		case 1: tar_angle_temp = 90; break;    //	32
		case 2: tar_angle_temp = 180; break;   //	302
		case 3: tar_angle_temp = 270; break;   //	212

		default :tar_angle_temp = 0; break;
	}
	return tar_angle_temp;
}

/**
 * @brief 检测要发送的数据
 * 
 * @param TxData 发送缓存区
 * @return * void 
 */
void check_TxData(uint8_t *TxData)
{
	uint8_t buf[LENGTH] = {0};
	uint8_t *p = NULL;
	uint8_t length = 0;
	p = buf;
	uint16_t CRC16 = 0;
	
	
	*p++ = 0x02;
	*p++ = 0x00;
	*p++ = 0x10;
	*p++ = 0x04;
	*p++ = 0x01;
	*p++ = UART1_Tx_Buf[5];
	*p++ = UART1_Tx_Buf[6];
	*p++ = UART1_Tx_Buf[7];
	*p++ = UART1_Tx_Buf[8];
	*p++ = UART1_Tx_Buf[9];
	*p++ = UART1_Tx_Buf[10];
	*p++ = UART1_Tx_Buf[11];
	*p++ = UART1_Tx_Buf[12];
	*p++ = UART1_Tx_Buf[13];
	*p++ = UART1_Tx_Buf[14];
	*p++ = UART1_Tx_Buf[15];
	*p++ = UART1_Tx_Buf[16];

	buf[1] = p - buf;
	
	CRC16 = modbus_crc16(buf, buf[1]);
	*p++ = CRC16 & 0xFF;
	*p++ = CRC16 >> 8;
	length = p - buf;
	inset_esc(buf, &length);
	memcpy(UART1_Tx_Buf_reg, buf, length);
	HAL_UART_Transmit(&huart1, buf, length, 0xFFFF);
}

/**
 * @brief 	check receive data
 * @param	pData	The first address of the destination location
 * @param	len		Number of valid data
 * @retval
 */	
void Check_RecData(unsigned char *pData ,unsigned len)
{
	Uint16ToByte_Typedef Uint16ToByte_CRC16;
	delete_esc(delete_esc_data, UART1_Rx_Buf_reg, &len );
	
	if (len > 0x20)
	{
		clean_data();
	}
	else
	{
		if( (delete_esc_data[0] == UART_addr) && (delete_esc_data[2] == 0x11) && (delete_esc_data[3] == 0x01) && (delete_esc_data[4] == 0x03))
		{
			if( len == delete_esc_data[1] + 2 )
			{
				Uint16ToByte_CRC16.data[0] = delete_esc_data[delete_esc_data[1]];
				Uint16ToByte_CRC16.data[1] = delete_esc_data[delete_esc_data[1] + 1];
				if((Uint16ToByte_CRC16.data_uint16_t == CRC16_Modbus(delete_esc_data, delete_esc_data[1])) && (CRC16_Modbus(delete_esc_data, delete_esc_data[1] ) != 0))
				{
					update_flag_typedef();
					//update_flag();
					getkey(delete_esc_data);
					update_rec_motor_para(&gw_move, &device_move, &device_and_gw_move, &device_action);
					update_inject_symbol(&inject_action);
					
					Action_Detect(&rs485_key, &rs485_key_pre, &rs485_action);
					clean_data();
				}
				clean_data();
			}
		else 
		clean_data();
		}
		else 
		clean_data();
	}
}

/**
 * @brief clear buffer data
*/
void clean_data(void)
{
	memset(UART1_Rx_Buf_reg, 0, sizeof(UART1_Rx_Buf_reg)); 
//	memset(delete_esc_data, 0, sizeof(delete_esc_data));
	UART1_Rx_cnt = 0;
	UART1_Rx_flg = 0;		
}




void uart_step_inital(void)   ///  55 AA  {header [2],length [1],ID[1],CMD[1],Index[1] ,DATA[N] CHECK_SUM[1]}  
{
	Uartmotor3_to_max[0] = 0X55;
	Uartmotor3_to_max[1] = 0XAA; 
	Uartmotor3_to_max[2] = 0X04;               ///length = N+2   CMD  +index 
	Uartmotor3_to_max[3] = 0X01;								///  change with software on pc
	Uartmotor3_to_max[4] = 0X21; 							///cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_to_max[5] = 0X37;								///reg address     2
	Uartmotor3_to_max[6] = Uartmotor3_maxtarget % 256;
	Uartmotor3_to_max[7] = Uartmotor3_maxtarget / 256;
	Uartmotor3_to_max[8] = 0X5D + Uartmotor3_maxtarget % 256 + Uartmotor3_maxtarget / 256;

	Uartmotor3_to_min[0] = 0X55;
	Uartmotor3_to_min[1] = 0XAA;
	Uartmotor3_to_min[2] = 0X04;               ///length = N+2   CMD  +index 
	Uartmotor3_to_min[3] = 0X01;								///  change with software on pc
	Uartmotor3_to_min[4] = 0X21; 							///cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_to_min[5] = 0X37;								///reg address     2
	Uartmotor3_to_min[6] = Uartmotor3_mintarget % 256;
	Uartmotor3_to_min[7] = Uartmotor3_mintarget / 256;
	Uartmotor3_to_min[8] = (uint8_t)(0X5D + Uartmotor3_mintarget % 256 + Uartmotor3_mintarget / 256);
	
	Uartmotor3_target[0] = 0X55;
	Uartmotor3_target[1] = 0XAA; 
	Uartmotor3_target[2] = 0X04;               ///length = N+2   CMD  +index 
	Uartmotor3_target[3] = 0X01;								///  change with software on pc
	Uartmotor3_target[4] = 0X21; 							///cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_target[5] = 0X37;								///reg address     2
	Uartmotor3_target[6] = 0;
	Uartmotor3_target[7] = 0;
	Uartmotor3_target[8] = 0;	
}

void uart_step_read(void)   ///  55 AA  {header [2],length [1],ID[1],CMD[1],Index[1] ,DATA[N] CHECK_SUM[1]}  
{
	Uartmotor3_position[0] = 0X55;
	Uartmotor3_position[1] = 0XAA; 
	Uartmotor3_position[2] = 0X03;               ///length = N+2   CMD  +index 
	Uartmotor3_position[3] = 0X01;								///  change with software on pc
	Uartmotor3_position[4] = 0X01; 							///cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_position[5] = 0X37;								///reg address     2
	Uartmotor3_position[6] = 0x02;
	Uartmotor3_position[7] = 0x3E;		//check sum

	HAL_UART_Transmit(&huart3, Uartmotor3_position, sizeof(Uartmotor3_position), 10);
	while(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) != SET);		//等待发送结束
}

	

void bsp_RunPer10ms(void)
{	
	bsp_KeyScan10ms();

	PCA9539_DigitalRead(&PCA9555_KEY, 0, &bit_value);
	Update_Key_status(&PCA9555_KEY, &PCA9555_LED);
	pca9555_decode(&PCA9555_KEY);
	Update_I2CLed(&PCA9555_KEY, &PCA9555_LED);		
	key_update_flag = 0;
	
}

void bsp_RunPer50ms(void)
{
	Excute_Usart_TxData();
	
	AS5048B_Read(&as5048b00, ADDR_A, &angle_as5048b_h);
	AS5048B_Read(&as5048b01, ADDR_B, &angle_as5048b_h);
	AS5048B_Read(&as5048b10, ADDR_C, &angle_as5048b_h);
	AS5048B_Read(&as5048b11, ADDR_D, &angle_as5048b_h);
   

	motor_real_speed(&as5048b00);
	motor_real_speed(&as5048b01);

   	high_transition_low(&DCmotor1, &as5048b00);
	high_transition_low(&DCmotor2, &as5048b01);
    cal_wire_device_length(&as5048b00);
	cal_wire_device_length(&as5048b01);
}

void bsp_RunPer100ms(void)
{
	data_collection(&status_monitor);
	
#if INJECT
	// bsp_hsppadx4_GetPressureInPa(&hsppad147_handle);	//Update_Motor_Status and bsp_hsppadx4_GetPressureInPa have conflict
//	pressure_measure_read(&xgzp6877d_handle);
	Pressure_Temperature_Cal(&xgzp6877d_handle);

	ercp_inject_feedback(&status_monitor, UART1_Tx_Buf, &SPICS1);
	ercp_inject_feedback(&status_monitor, UART1_Tx_Buf, &SPICS2);
	ercp_inject_feedback(&status_monitor, UART1_Tx_Buf, &SPICS3);
#else
	ercp_switch_feedback(&status_monitor, UART1_Tx_Buf);
	ercp_device_feedback(&status_monitor, UART1_Tx_Buf);
	ercp_gw_feedback(&status_monitor, UART1_Tx_Buf);
#endif
    

	
}

void bsp_RunPer200ms(void)
{ 
	

}

void bsp_RunPer500ms(void)
{
	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
	HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
	HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
}





