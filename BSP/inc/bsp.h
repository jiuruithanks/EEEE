#ifndef __BSP_H 
#define __BSP_H
#include "main.h"
#include "bsp_as5048b.h"
#include "bsp_key.h"
#include "usart.h"
#include <string.h>
#include "crc.h"


/******************************************************** 宏定义 ***************************************************************/
#define Uartmotor3_mintarget 500
#define Uartmotor3_maxtarget 1900


#define RECEIVE_DATA_GATE_L 97
#define RECEIVE_DATA_GATE_H 158

	 
#define STATE_IDLE 	0x00	 
#define STATE_RUP  	0x01
#define STATE_RDOWN 0x02
#define STATE_RPLUS 0x03
#define STATE_RSUB  0x04


#define Pi 				3.1415926
#define R 				6.5
#define CON_VAL_LESS 	5
#define CON_VAL_MORE 	2000

/******************************************************** 结构体 ***************************************************************/

typedef union
{
	signed		short		int			data_int16_t;
	unsigned					char		data[2];
}Int16ToByte_Typedef;

typedef union
{
	unsigned	short		int			data_uint16_t;
	unsigned					char		data[2];
}Uint16ToByte_Typedef;

typedef union
{
	unsigned					int			data_uint32_t;
	unsigned					char		data[4];
}Uint32ToBit_Typedef;



typedef struct
{
	uint8_t	up_flag ;				//
	uint8_t	down_flag ;			//
	uint8_t	right_flag ;				//
	uint8_t	left_flag ;		//

	uint8_t	triangle_flag;				//
	uint8_t	circle_flag ;		//
	uint8_t	cross_flag ;				//
	uint8_t	square_flag ;			//
		
	uint8_t	L1_flag;				//
	uint8_t	L2_flag;		//
	uint8_t	L3_flag ;				//
	uint8_t	R1_flag ;			//
	uint8_t	R2_flag ;				//
	uint8_t	R3_flag ;		//
	uint8_t	SHARE_flag ;			//
	uint8_t	OPTIONS_flag ;		//

	uint8_t	L3_L_flag ;				//
	uint8_t	L3_R_flag;			//
	uint8_t	L3_UP_flag ;				//
	uint8_t	L3_DOWN_flag ;			//

	uint8_t	R3_L_flag ;				//
	uint8_t	R3_R_flag ;			//
	uint8_t	R3_UP_flag ;				//
	uint8_t	R3_DOWN_flag ;			//

		
	uint8_t	dec_head_flag ;				//
	uint8_t	dec_back_flag ;			//
	uint8_t	dec_left_flag ;				//
	uint8_t	dec_right_flag ;		//

	uint8_t	backup0_flag;				//
	uint8_t	backup1_flag;		//
	uint8_t	backup2_flag ;				//
	uint8_t	backup3_flag ;			//
	uint8_t	backup4_flag ;				//
	uint8_t	backup5_flag ;		//
	uint8_t	backup6_flag ;			//
	uint8_t	backup7_flag ;		//

	uint8_t	backup8_flag;				//
	uint8_t	backup9_flag;		//
	uint8_t	backup10_flag ;				//
	uint8_t	backup11_flag ;			//
	uint8_t	backup12_flag ;				//
	uint8_t	backup13_flag ;		//
	uint8_t	backup14_flag ;			//
	uint8_t	backup15_flag ;		//
	
}RS485_KEY_T;

typedef struct
{
	uint8_t	up_flag ;				//
	uint8_t	down_flag ;			//
	uint8_t	right_flag ;				//
	uint8_t	left_flag ;		//

	uint8_t	triangle_flag;				//
	uint8_t	circle_flag ;		//
	uint8_t	cross_flag ;				//
	uint8_t	square_flag ;			//
		
	uint8_t	L1_flag;				//
	uint8_t	L2_flag;		//
	uint8_t	L3_flag ;				//
	uint8_t	R1_flag ;			//
	uint8_t	R2_flag ;				//
	uint8_t	R3_flag ;		//
	uint8_t	SHARE_flag ;			//
	uint8_t	OPTIONS_flag ;		//

	uint8_t	L3_L_flag ;				//
	uint8_t	L3_R_flag;			//
	uint8_t	L3_UP_flag ;				//
	uint8_t	L3_DOWN_flag ;			//

	uint8_t	R3_L_flag ;				//
	uint8_t	R3_R_flag ;			//
	uint8_t	R3_UP_flag ;				//
	uint8_t	R3_DOWN_flag ;			//

		
	uint8_t	dec_head_flag ;				//
	uint8_t	dec_back_flag ;			//
	uint8_t	dec_left_flag ;				//
	uint8_t	dec_right_flag ;		//

	uint8_t	backup0_flag;				//
	uint8_t	backup1_flag;		//
	uint8_t	backup2_flag ;				//
	uint8_t	backup3_flag ;			//
	uint8_t	backup4_flag ;				//
	uint8_t	backup5_flag ;		//
	uint8_t	backup6_flag ;			//
	uint8_t	backup7_flag ;		//

	uint8_t	backup8_flag;				//
	uint8_t	backup9_flag;		//
	uint8_t	backup10_flag ;				//
	uint8_t	backup11_flag ;			//
	uint8_t	backup12_flag ;				//
	uint8_t	backup13_flag ;		//
	uint8_t	backup14_flag ;			//
	uint8_t	backup15_flag ;		//
	
}RS485_KEY_ACTION_T;

typedef struct DEVICE_GW
{
	uint8_t rec_data;	//接收数据
	uint8_t run_dir;
	uint8_t run_uartflag;
	uint32_t run_speed;

	uint8_t gw_dev_flag;	//导丝进，器械退
	uint8_t dev_gw_flag;	//导丝退，器械进
	uint8_t motor_auto_run_flag;
	uint8_t motor_move_head_flag; //导丝和器械进退相关标识
	uint8_t motor_move_back_flag;
}DEVICE_GW_Typedef;


/******************************************************** 提供外部变量 ***************************************************************/

extern uint8_t bsp_RunPer10ms_flag;
extern uint8_t bsp_RunPer50ms_flag;
extern uint8_t bsp_RunPer100ms_flag;
extern uint8_t bsp_RunPer200ms_flag;
extern uint8_t bsp_RunPer500ms_flag;

extern uint8_t Uartmotor3_to_min[9];
extern uint8_t Uartmotor3_to_max[9];
extern uint8_t Uartmotor3_target[9];
extern uint8_t Uartmotor3_position[9];

extern RS485_KEY_T rs485_key,rs485_key_pre;
extern RS485_KEY_ACTION_T rs485_action;
extern uint8_t state_change_flag;
extern DEVICE_GW_Typedef device_move, device_action, gw_move, device_and_gw_move, inject_action, encoder_select;

extern uint16_t angle_as5048b_h;
extern uint8_t bit_value;
/******************************************************** 函数声明 ***************************************************************/
/* 初始化及标志位*/
void init_rs485_key(void);
uint8_t Get_Mode(void);
void update_flag(void);
void Action_Detect(RS485_KEY_T *key, RS485_KEY_T *key_pre, RS485_KEY_ACTION_T *action);
void update_flag_typedef(void);
void update_rec_motor_para(DEVICE_GW_Typedef *gw_movex, DEVICE_GW_Typedef *device_movex, DEVICE_GW_Typedef *device_and_gw_movex, DEVICE_GW_Typedef *device_actionx);
void update_inject_symbol(DEVICE_GW_Typedef *inject_action);
uint16_t change_target_angle(uint8_t tar_pos);

/* 通信协议相关 */
void delete_esc(unsigned char *data, unsigned char *data_esc, unsigned int *length);
void inset_esc(unsigned char *data, unsigned char *length);
void getkey(unsigned char *delete_esc_data);
void check_TxData(uint8_t *TxData);
void Check_RecData(unsigned char *pData ,unsigned len);
void clean_data(void);
void uart_step_inital(void);
void uart_step_read(void);
// void usart3_protocol(uint8_t func, uint8_t mode, uint8_t spix);



/* 循环 */
void bsp_RunPer10ms(void);
void bsp_RunPer50ms(void);
void bsp_RunPer100ms(void);
void bsp_RunPer200ms(void);
void bsp_RunPer500ms(void);

#endif
