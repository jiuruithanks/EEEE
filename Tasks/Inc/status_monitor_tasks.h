#ifndef __STATUS_MONITOR_TASKS_H
#define __STATUS_MONITOR_TASKS_H
#include <stdint.h>
#include "bsp_as5048b.h"
#include "tmc5160.h"

/*********************************************************************** 宏定义 ******************************************************************************/
#define WORK_FIRST		305
#define WORK_SECOND		215
#define WORK_THIRD		125
#define WORK_ROURTH		35

#define ERCP_SWITCH 	as5048b01
#define ERCP_DEVICE		DCmotor2	
#define ERCP_GW			DCmotor1

#define POS_FIRST		0X08
#define POS_SECOND		0X07
#define POS_THIRD		0X06
#define POS_FOURTH		0X05

/*********************************************************************** 数据类型 ******************************************************************************/

typedef enum _in_place
{
	no_reach,
	reach
}IN_PLACE;

typedef enum _clutch
{
	open,
	close
}CLUTCH;

typedef enum _motor_status
{
	motor_stop = 0x00,
	motor_head,	
	motor_back,
	motor_error
}MOTOR_STATUS;

typedef enum _device_selection
{
	cutter = 1,		//切开刀
	sacculus,		//球囊
	net_basket,		//取石网篮
	drainage_tube,	//胆管引流管
	biliary_stent	//胆管支架
}DEVICE_SELECTION;

typedef enum _inject_selection
{
	attract = 1,	//吸引
	water,			//送水
	radiography,	//造影
	air				//打气
}INJECT_SELECTION;


typedef struct _status_monitor
{
    float 	angle_val_f;            //磁传感器角度值
    uint8_t encoder;                //编码器档位
    uint8_t gw_status;              //导丝状态
    uint8_t device_status;          //器械状态
    uint8_t gw_device_status;       //导丝和器械组合进退
	uint8_t dev_control_status;		//器械控制
	uint8_t inject_select_status;	//注射库选择
    uint8_t inject_control_status;  //注射库控制
}STATUS_MONITOR;

/*********************************************************************** 外部变量 ******************************************************************************/
extern STATUS_MONITOR status_monitor;
extern uint8_t g_usart_flag;

/*********************************************************************** API ******************************************************************************/
void mode_select(uint8_t mode);
void data_collection(STATUS_MONITOR *status);
void ercp_switch_feedback(STATUS_MONITOR *status, uint8_t *data);
void ercp_device_feedback(STATUS_MONITOR *status, uint8_t *data);
void ercp_gw_feedback(STATUS_MONITOR *status, uint8_t *data);
void ercp_inject_feedback(STATUS_MONITOR *status, uint8_t *data, SPI_CS_TypeDef *CS);
void Excute_Usart_TxData(void);
#endif
