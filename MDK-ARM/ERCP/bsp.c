#include "bsp.h"
#include "pca9539.h"
#include "encoder.h"

uint8_t bsp_RunPer10ms_flag=0;
uint8_t bsp_RunPer100ms_flag=0;
uint8_t bsp_RunPer500ms_flag=0;

uint8_t  pre_pre_state;
uint8_t  pre_state;
uint8_t  current_state;
uint8_t	 manual_head_speed;
uint8_t	 manual_back_speed;

RS485_KEY_T rs485_key,rs485_key_pre;
RS485_KEY_ACTION_T  rs485_action;
DEVICE_GW_Typedef device_move, device_action, gw_move, device_and_gw_move;
//DEVICE_GW_COMBINED_Typedef device_and_gw_action;

unsigned char manual_head_data[10];
unsigned char manual_back_data[10];
unsigned char manual_stop_data[10];
 
uint16_t Crc_data_cal_h;
uint16_t Crc_data_cal_b;
uint16_t Crc_data_cal_s;

uint8_t inspire_motor_flag=0; 
uint8_t state_change_flag=0;

uint8_t auto_manual_control_flag=0;      ////自动、手动模式标志位
uint8_t manual_stop_motor_flag;   ///初次时，停止所有电机  再循环时，为1 不再停止 电机
uint8_t auto_stop_motor_flag;

unsigned char Uartmotor3_to_max[9];
unsigned char Uartmotor3_to_min[9];
unsigned char Uartmotor3_target[9];		
unsigned char Uartmotor3_position[9];	
// unsigned char Uartmotor3_real[9];		


uint16_t angle_as5048b_h;
uint8_t bit_value;

uint32_t  press_for_test;
uint32_t speed_for_test;

uint8_t UART_addr=2;




void init_rs485_key()
{
	RS485_KEY_T rs485_key={0};
	RS485_KEY_T	rs485_key_pre={0};
}

uint8_t Get_Mode(void)
{
    uint8_t mode;
    mode=  HAL_GPIO_ReadPin(CPU_MODE0_GPIO_Port,CPU_MODE0_Pin)|(HAL_GPIO_ReadPin(CPU_MODE1_GPIO_Port,CPU_MODE1_Pin)<<1);
	return mode;
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
	unsigned char i = 0,j = 0;
	for( i = 0 ,j = 0 ; i < *length ; i++,j++ )
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
	unsigned char i = 0,j = 0;
	
	memcpy(buf,data,*length);
	
	for( i = 1 ,j = 1 ; i < *length ; i++,j++ )
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

/**
 * @brief 	check receive data
 * @param	pData	The first address of the destination location
 * @param	len		Number of valid data
 * @retval
 */	
void Check_RecData(unsigned char *pData ,unsigned len)
{
	Uint16ToByte_Typedef Uint16ToByte_CRC16;
	delete_esc(delete_esc_data ,UART1_Rx_Buf_reg ,&len );
	
	if( (delete_esc_data[0] == UART_addr)&&(delete_esc_data[2]==0x11)&&(delete_esc_data[3]==0x01)&&(delete_esc_data[4]==0x03))
	{
		if( len == delete_esc_data[1] + 2 )
		{
			Uint16ToByte_CRC16.data[0] = delete_esc_data[ delete_esc_data[1] ];
			Uint16ToByte_CRC16.data[1] = delete_esc_data[ delete_esc_data[1] + 1 ];
			if((Uint16ToByte_CRC16.data_uint16_t == CRC16_Modbus( delete_esc_data, delete_esc_data[1] ))&&(CRC16_Modbus( delete_esc_data, delete_esc_data[1] )!=0))
			{
       			update_flag_typedef();
				//update_flag();
        		getkey(delete_esc_data);
				update_rec_motor_para(&device_move, &device_action, &gw_move, &device_and_gw_move);

				Action_Detect(& rs485_key,&rs485_key_pre,&rs485_action);
				clean_data();
			}
		}
    else 
    clean_data();
	}
  	else 
  	clean_data();
}

/**
 * @brief clear buffer data
*/
void clean_data(void)
{
	memset(UART1_Rx_Buf_reg,0,sizeof(UART1_Rx_Buf_reg)); 
	memset(  delete_esc_data,0,sizeof(delete_esc_data));
	UART1_Rx_cnt = 0;
	UART1_Rx_flg = 0;		
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

void Action_Detect(RS485_KEY_T * key ,RS485_KEY_T * key_pre ,RS485_KEY_ACTION_T *action)
{
uint8_t* p_pre = key;
uint8_t* p 	= key_pre;	
uint8_t* p_t  	= action;
uint8_t i;
	for(i=0; i<sizeof(RS485_KEY_T)/sizeof(uint8_t); i++)
	{
		if((*(p_pre++)==0)&&((*p++)==1) )
		(* p_t++)++;

		if(*p_t>2)
		*p_t=0;
	}

} 

void update_flag_typedef(void)
{
	uint8_t * p_pre = &rs485_key_pre;
	uint8_t * p 	= &rs485_key;

	uint8_t i;
	for(i=0; i<sizeof(RS485_KEY_T)/sizeof(uint8_t); i++)
	{
		*(p_pre++)=(*p++);
	}
}

void uart_step_inital(void)   ///////  55 AA  {header [2],length [1],ID[1],CMD[1],Index[1] ,DATA[N] CHECK_SUM[1]}  
{
	Uartmotor3_to_max[0]=0X55;
	Uartmotor3_to_max[1]=0XAA; 
	Uartmotor3_to_max[2]=0X04;               //////length = N+2   CMD  +index 
	Uartmotor3_to_max[3]=0X01;								////  change with software on pc
	Uartmotor3_to_max[4]=0X21; 							//////cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_to_max[5]=0X37;								/////reg address     2
	Uartmotor3_to_max[6]=Uartmotor3_maxtarget%256;
	Uartmotor3_to_max[7]=Uartmotor3_maxtarget/256;
	Uartmotor3_to_max[8]=0X5D+Uartmotor3_maxtarget%256+Uartmotor3_maxtarget/256;

	Uartmotor3_to_min[0]=0X55;
	Uartmotor3_to_min[1]=0XAA; 
	Uartmotor3_to_min[2]=0X04;               //////length = N+2   CMD  +index 
	Uartmotor3_to_min[3]=0X01;								////  change with software on pc
	Uartmotor3_to_min[4]=0X21; 							//////cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_to_min[5]=0X37;								/////reg address     2
	Uartmotor3_to_min[6]=Uartmotor3_mintarget%256;
	Uartmotor3_to_min[7]=Uartmotor3_mintarget/256;
	Uartmotor3_to_min[8]=0X5D+Uartmotor3_mintarget%256+Uartmotor3_mintarget/256;
	
	Uartmotor3_target[0]=0X55;
	Uartmotor3_target[1]=0XAA; 
	Uartmotor3_target[2]=0X04;               //////length = N+2   CMD  +index 
	Uartmotor3_target[3]=0X01;								////  change with software on pc
	Uartmotor3_target[4]=0X21; 							//////cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_target[5]=0X37;								/////reg address     2
	Uartmotor3_target[6]=0;
	Uartmotor3_target[7]=0;
	Uartmotor3_target[8]=0;	
}

void uart_step_read(void)   ///////  55 AA  {header [2],length [1],ID[1],CMD[1],Index[1] ,DATA[N] CHECK_SUM[1]}  
{
	Uartmotor3_position[0]=0X55;
	Uartmotor3_position[1]=0XAA; 
	Uartmotor3_position[2]=0X03;               //////length = N+2   CMD  +index 
	Uartmotor3_position[3]=0X01;								////  change with software on pc
	Uartmotor3_position[4]=0X01; 							//////cmd mode  0x21 =position  mode    0x01 read  ,0x02 write  0x03 write without back 
	Uartmotor3_position[5]=0X37;								/////reg address     2
	Uartmotor3_position[6]=0x02;
	Uartmotor3_position[7]=0x3E;		//check sum

	HAL_UART_Transmit(&huart3 ,Uartmotor3_position, sizeof(Uartmotor3_position),  10);
	while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)!=SET);		//等待发送结束
}

//void update_uart_motor_target( uint16_t target )
//{
//	uint16_t target_temp =target ;
//	Uartmotor3_target[6]=target_temp%256;
//	Uartmotor3_target[7]=target_temp/256;
//	Uartmotor3_target[8]=0X5D+target_temp%256+target_temp/256;

//	HAL_UART_Transmit(&huart3 ,Uartmotor3_target, sizeof(Uartmotor3_target),  10);
//	{
//		while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)!=SET);		//等待发送结束
//	}
//}

/// @brief ///ERCP  used to receive message from extend board of platform 
/// to contorl device control /gw fwd /bwd forceps lifting .
/// @param delete_esc_data 
void getkey(unsigned char *delete_esc_data )
{
	unsigned char *p = delete_esc_data;
	device_move.rec_data = p[5];
	device_action.rec_data = p[6];
	gw_move.rec_data= p[7] ;
	device_and_gw_move.rec_data = p[9];
	
//	device_move.move_head_uartflag=p[9];
//	device_move.move_back_uartflag=p[10];
}	

/// @brief Knob control of the flag bit change
/// @param device_actionx 			control uart motor action
/// @param gw_movex       			contorl gw action 
/// @param device_movex   			contorl device action
/// @param device_and_gw_movex   	contorl device and gw action
void update_rec_motor_para(DEVICE_GW_Typedef *device_movex, DEVICE_GW_Typedef *device_actionx,DEVICE_GW_Typedef *gw_movex, DEVICE_GW_Typedef *device_and_gw_movex)
{
	//device action
	if (device_movex->rec_data<RECEIVE_DATA_GATE_L)
	{
		device_movex->run_dir=0;
		device_movex->run_uartflag=1;
		device_movex->run_speed=abs(127-device_movex->rec_data)*100/127;

	}
	else if (device_movex->rec_data>RECEIVE_DATA_GATE_H)
	{
		device_movex->run_dir=1;
		device_movex->run_uartflag=1;
		device_movex->run_speed=abs(device_movex->rec_data-128)*100/127;
	}
	else 
	{
		device_movex->run_uartflag=0;
		device_movex->run_speed=0;
	}
	//gw action
	if (gw_movex->rec_data<RECEIVE_DATA_GATE_L)
	{
		gw_movex->run_dir=0;
		gw_movex->run_uartflag=1;
		
		gw_movex->run_speed=abs(127-gw_movex->rec_data)*100/127;
	}
	else if (gw_movex->rec_data>RECEIVE_DATA_GATE_H)
	{
		gw_movex->run_dir=1;
		gw_movex->run_uartflag=1;
		gw_movex->run_speed=abs(gw_movex->rec_data-128)*100/127;
	}
	else 
	{
		gw_movex->run_uartflag=0;
		gw_movex->run_speed=0;
	}
	//uart motor action
	if (device_actionx->rec_data<RECEIVE_DATA_GATE_L)
	{
		device_actionx->run_dir = 0;
		device_actionx->run_uartflag = 1;
		device_actionx->run_speed = abs(127 - device_actionx->rec_data) * 100 / 127;
	}
	else if (device_actionx->rec_data>RECEIVE_DATA_GATE_H)
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
	//gw and device combined action 
	if (device_and_gw_movex->rec_data < RECEIVE_DATA_GATE_L)
	{
		device_and_gw_movex->run_dir = 0;
		device_and_gw_movex->run_uartflag = 1;
		device_and_gw_movex->run_speed = abs(127 - device_movex->rec_data) * 100 / 127;
		
	}
	else if (device_and_gw_movex->rec_data > RECEIVE_DATA_GATE_H)
	{
		device_and_gw_movex->run_dir = 1;
		device_and_gw_movex->run_uartflag = 1;
		device_and_gw_movex->run_speed = abs(device_movex->rec_data - 128) * 100 / 127;
	}
	else 
	{
		device_and_gw_movex->run_uartflag = 0;
		device_and_gw_movex->run_speed = 0;
	}
}


 
//////用于保存器械切换位置
uint8_t channel_temp;
void Change_Device(SPI_CS_TypeDef* CS, PCA9555_HandleTypeDef* key )
{
	SPI_CS_TypeDef *p= CS;
	PCA9555_HandleTypeDef * key_temp=key;

	switch (key_temp->rotate)
	{
		case  8 : channel_temp=0;break;
		case  7 : channel_temp=1;break;
		case  6 : channel_temp=2;break;
		case  5 : channel_temp=3;break;
		case  4 : channel_temp=3;break;
	default : channel_temp=channel_temp;break;
	}
	Run_STEP_Motor_POS(p,channel_temp);
}

DIR_NEW_MOTOR dir_new;
// IS_ARRIVE_POS assign_pos;
// IS_ARRIVE_POS assign_pos01;
// uint8_t arrive_pos;							//判断导丝和器械是否到位
// uint8_t arrive_pos_pre;
// uint16_t pos_head_move_ori;					//当前位置
// uint16_t pos_head_move_ori_pre;				//上一次的值
// uint32_t pos_head_move_base;				//第一次读取到的值为基准
// uint32_t pos_head_move_after_zero_process;	//累加值
/*
*@brief		实现导丝和器械电机到达指定位置
*@param		直流电机的选择
*@return	void
*/
void high_transition_low(DCmotor_TypeDef* DCmotor, AS5048B_HandleTypeDef* as5048bxx)
{
  //AS5048B_Read(as5048bxx, 0xFE, &angle_as5048b_h);
	// arrive_pos=HAL_GPIO_ReadPin(DCmotor->head_limit_GPIOx,DCmotor->head_limit_GPIO_Pin);  
	as5048bxx->arrive_pos=HAL_GPIO_ReadPin(DCmotor->head_limit_GPIOx,DCmotor->head_limit_GPIO_Pin); 

	// as5048bxx->Cycle = 0;
	// as5048bxx->length = 0;
	// as5048bxx->sum_length = 0;
	if (as5048bxx->arrive_pos_pre == 1)
	{
		// as5048bxx->Cycle = 0;
		if (as5048bxx->arrive_pos == 0)
		{
			as5048bxx->pos_head_move_base = as5048bxx->angle_h;
			as5048bxx->pos_temp = 0;
			as5048bxx->pos_final = 0;
			as5048bxx->Cycle = 0;
			as5048bxx->length = 0;
			as5048bxx->sum_length = 0;
		}
	}
//	as5048bxx->arrive_pos_pre = as5048bxx->arrive_pos;
	
	// if (as5048bxx->arrive_pos_pre == 0)
	// {
	// 	if (as5048bxx->arrive_pos == 1)
	// 	{
	// 		as5048bxx->pos_head_move_base = as5048bxx->angle_h;
	// 		as5048bxx->Cycle = 0;
	// 		as5048bxx->length = 0;
	// 		as5048bxx->sum_length = 0;
	// 	}
	// }
	as5048bxx->arrive_pos_pre = as5048bxx->arrive_pos;
}

/*
*@brief		计算导丝和器械的进退长度
*@param		直流电机的选择
*@return	void
*/
void cal_wire_device_length(DCmotor_TypeDef *DCmotor, AS5048B_HandleTypeDef *as5048bxx, DIR_NEW_MOTOR *dir_new)
{
	// uint8_t cnt = 0;
	dir_new->dir_wire = DCmotor->dir ^ 0;		//^0为正向 
	dir_new->dir_devcie = DCmotor->dir ^ 1;		//^1为反向 
	if (as5048bxx == &as5048b00)
	{
		as5048bxx->pos_head_move_ori = dir_new->dir_wire ? (0x3fff - as5048bxx->angle_h) : (as5048bxx->angle_h); //judge direction 
	}
	if (as5048bxx == &as5048b01)
	{
		as5048bxx->pos_head_move_ori = dir_new->dir_devcie ? (0x3fff - as5048bxx->angle_h) : (as5048bxx->angle_h); //judge direction
	}
	
	// pos_head_move_ori = DCmotor->dir ? (0x3fff - as5048bxx->angle_h) : (as5048bxx->angle_h); //judge direction 
	if (((as5048bxx->pos_head_move_ori & 0x3000) == 0x3000) && ((as5048bxx->pos_head_move_ori_pre & 0x3000) == 0))	//back
	{																					//(& 0xffffc000) 保留周期数（如时钟12点 9 + 12 = 21 取成 12）
		as5048bxx->pos_head_move_after_zero_process = ((as5048bxx->pos_head_move_after_zero_process - 0x4000) & 0xffffc000) + as5048bxx->pos_head_move_ori; //周期数 + pos_head_move_ori
	}
	else if (((as5048bxx->pos_head_move_ori & 0x3000) == 0) && ((as5048bxx->pos_head_move_ori_pre & 0x3000) == 0x3000)) //head
	{
		as5048bxx->pos_head_move_after_zero_process = ((as5048bxx->pos_head_move_after_zero_process + 0x4000) & 0xffffc000) + as5048bxx->pos_head_move_ori; //周期数 + pos_head_move_ori
	} 
	else 
	{	//不过零点的处理
		as5048bxx->pos_head_move_after_zero_process = ((as5048bxx->pos_head_move_after_zero_process + 0x0000) & 0xffffc000) + as5048bxx->pos_head_move_ori; //周期数 + pos_head_move_ori
	}

	as5048bxx->pos_head_move_ori_pre = as5048bxx->pos_head_move_ori;
	// 更新当前位置，过零点处理后的数据，减去起始位置，有可能是负数k
	as5048bxx->pos_temp = as5048bxx->pos_head_move_after_zero_process - as5048bxx->pos_head_move_base;
	
	
	as5048bxx->Cycle = as5048bxx->pos_temp / 0x4000; 
	as5048bxx->pos_final = (as5048bxx->pos_temp % 0x4000);

	// Cycle = (pos_head_move_ori - pos_head_move_base) / 0x4000;
	// DCmotor->pos_final = (pos_head_move_ori - pos_head_move_base) % 0x4000;
	
	as5048bxx->length = ((as5048bxx->pos_final) * Pi * 6.5 / 0x2000);//轮子直径13  弧长
	as5048bxx->sum_length = (as5048bxx->Cycle * (2 * Pi * 6.5)) + as5048bxx->length;//总周长

	if (as5048bxx->arrive_pos_pre == 1)
	{
		if (as5048bxx->arrive_pos == 0)
		{
			as5048bxx->pos_head_move_base = as5048bxx->angle_h;
			// as5048bxx->Cycle = 0;
			as5048bxx->length = 0;
			as5048bxx->sum_length = 0;
			// as5048bxx->pos_temp = 0;
			// as5048bxx->pos_final = 0;
		}
	}
}

// int32_t g_length_val;
// static uint8_t Control_EN = 1;
/*
*@brief		control gw & device move length
*@param		as5048bxx
*@return	void
*/
// void control_wireDevice_length(PCA9555_HandleTypeDef *hdev_key, AS5048B_HandleTypeDef* as5048bxx)
// {
// 	AS5048B_Read(as5048bxx, 0xFE, &angle_as5048b_h);
// 	//guide wire
// 	if(hdev_key->key_action[1] == 1 || hdev_key->key_action[7] == 1)
// 	{	
		
// 		if (as5048bxx->sum_length >= CON_VAL_LESS)
// 		// if ((as5048bxx->sum_length >= CON_VAL_LESS) || (as5048bxx->sum_length >= -40))
// 		{
// 			Run_STEP_Motor_SPEED( &SPICS1 ,1);	//guide wire move back
// 		}
// 		else
// 		{
// 			Stop_STEP_Motor( &SPICS1 );	
// 			hdev_key->key_action[1] = 0; 
// 			hdev_key->key_action[7] = 0;
// 			// Control_EN = 0;
// 		}	
	
// 	}
// 	else if((hdev_key->key_action[2] == 1 || hdev_key->key_action[12] == 1))
// 	{	
// 		if (as5048bxx->sum_length <= CON_VAL_MORE)
// 		{
// 			Run_STEP_Motor_SPEED( &SPICS1 ,0);	//guide wire move head
// 		}
// 		else
// 		{
// 			Stop_STEP_Motor( &SPICS1 );
// 			hdev_key->key_action[2] = 0;
// 			hdev_key->key_action[12] = 0;
// 			// Control_EN = 0;
// 		}		
// 	}
// 	else 
// 	{
// 		Stop_STEP_Motor( &SPICS1 );	
// 	}
// 	//device 
// 	if((hdev_key->key_action[3] == 1 || hdev_key->key_action[12] ==  1))
// 	{	
		
// 		if (as5048bxx->sum_length >= CON_VAL_LESS)
// 		{
// 			Run_STEP_Motor_SPEED( &SPICS2 ,1);	//device move back	
// 		}
// 		else
// 		{
// 			Stop_STEP_Motor( &SPICS2 );
// 			hdev_key->key_action[3] = 0;
// 			hdev_key->key_action[12] = 0;
// 			// Control_EN = 0;
// 		}
	
// 	}
// 	else if((hdev_key->key_action[4] == 1 || hdev_key->key_action[7] == 1))
// 	{
		
// 		if (as5048bxx->sum_length <= CON_VAL_MORE - 500)
// 		{
// 			Run_STEP_Motor_SPEED( &SPICS2 ,0);	//device move head
// 		}
// 		else
// 		{
// 			Stop_STEP_Motor( &SPICS2 );
// 			hdev_key->key_action[4] = 0;
// 			hdev_key->key_action[7] = 0;
// 			// Control_EN = 0;
// 		}
		
// 	}
// 	else 
// 	{
// 		Stop_STEP_Motor( &SPICS2 );	
// 	}
// 	// Control_EN = 1;
// }


uint8_t g_encoder_pos;
/*
****************************************************
*	函 数 名: Read_Encoder_DC
*	功能说明: 读取编码器档位来选择控制电机
* 形    参: 无
*	返 回 值: 无
****************************************************
*/
void Read_Encoder_Motor(void)
{
	g_encoder_pos = Read_protocol_encoder();

	switch (g_encoder_pos)
	{
	case 1:
		Panel_Step(Encoder_handler_buf[8], &SPICS1);
		Stop_DC_Motor(&DCmotor1);
    	Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	case 2:
		Panel_Step(Encoder_handler_buf[8], &SPICS3);
		Stop_DC_Motor(&DCmotor1);
    	Stop_STEP_Motor(&SPICS1);
		Stop_STEP_Motor(&SPICS2);
		break;
	case 3:
		Panel_Step(Encoder_handler_buf[8], &SPICS2);
		Stop_DC_Motor(&DCmotor1);
    	Stop_STEP_Motor(&SPICS1);
		Stop_STEP_Motor(&SPICS3);
		break;
	case 4:
		Panel_DC(Encoder_handler_buf[8]);
        Stop_STEP_Motor(&SPICS1);
        Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	default:
		Stop_DC_Motor(&DCmotor1);
		Stop_STEP_Motor(&SPICS1);
		Stop_STEP_Motor(&SPICS2);
		Stop_STEP_Motor(&SPICS3);
		break;
	}
	// if (Read_protocol_encoder() == 1)
	// {
	// 	Panel_DC(Read_protocol_injection());
	// }
	// else if (Read_protocol_encoder() == 2)
	// {
	// 	Panel_DC(Read_protocol_injection());
	// }
	// else if (Read_protocol_encoder() == 3)
	// {
	// 	/* code */
	// }
	// else if (Read_protocol_encoder() == 4)
 	// {
    // 	Panel_DC(Read_protocol_injection());
	// }
	// else 
	// {
	// 	Panel_DC(Read_protocol_injection());

	// }	 
}

void DC_Motor_balloon(void)
{
	Run_DC_Motor(&DCmotor4, 0, 12);
	HAL_Delay(2000);
	Stop_DC_Motor(&DCmotor4);

	Run_DC_Motor(&DCmotor4, 1, 12);
	HAL_Delay(2000);
	Stop_DC_Motor(&DCmotor4);
}

void bsp_RunPer10ms(void)
{

	bsp_KeyScan10ms();
	pca9555_digitalRead(&PCA9555_KEY,0,&bit_value);
	pca9555_decode(&PCA9555_KEY);
	Update_Key_status(&PCA9555_KEY,&PCA9555_LED);
	Update_I2CLed(&PCA9555_KEY,&PCA9555_LED);
	uart_step_read();
	Update_Motor_Status(&PCA9555_KEY,&gw_move,&device_move,&device_action,&device_and_gw_move);

//	
//	bsp_hsppadx4_GetPressureInPa(&hsppad147_handle);
//	press_for_test=	hsppad147_handle.m_Pressure-101325;
//	speed_for_test=Get_Speed(&SPICS1);
	
	AS5048B_Read(&as5048b00, 0xFE, &angle_as5048b_h) ;
 	AS5048B_Read(&as5048b01, 0xFE, &angle_as5048b_h) ;
	AS5048B_Read(&as5048b10, 0xFE, &angle_as5048b_h) ;
	AS5048B_Read(&as5048b11, 0xFE, &angle_as5048b_h) ;

	Change_Device (&SPICS3, & PCA9555_KEY );

}





void bsp_RunPer100ms(void)
{

}

void bsp_RunPer500ms(void)
{
	// if (IsTimeOut(&twinkleLedTimer, 100))
    // {
    //   HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    //   ResetTickTimer(&twinkleLedTimer);
    // }
	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
	HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
	HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
}


