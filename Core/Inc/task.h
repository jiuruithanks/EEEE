/**
 * @file task.h
 * @author Zhao Haofei
 * @brief
 * @version 1.0
 * @date 2023-06-14
 *
 * @copyright Copyright (c) 2023
 *
 */
 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#ifndef __TASKS_HEADER_
#define __TASKS_HEADER_
 
/**********************************************************************************************************************/
/* Macro Defines  																									  */
/**********************************************************************************************************************/

/* 来自主机发送给STM32的命令，用做调试、打印日志等 */
//#define PC_LOAD_LOG_PAGE    0x01
//#define PC_APPEND_LOG       0x02
//#define PC_LOG_ENABLE       0x03
//#define PC_LOG_DISABLE      0x04
#define PC_WRITE_EEPROM      0x05
//#define PC_STORE_LOG_NOW    0x06
//#define PC_GET_TIME         0x07
//#define PC_SET_TIME         0x08
//#define PC_READ_DS1302_REG  0x09
//#define PC_WRITE_DS1302_REG 0x0A
//#define PC_LOG_ADDR         0x0B
//#define PC_SET_TASK_FLAG    0x10
//#define PC_SECTOR_ERASE     0x20
//#define PC_BLOCK_ERASE      0xD8
//#define PC_CHIP_ERASE       0xC7
//#define PC_GREETING         0xFF

 
#define MAX_TASK_NUM         16
#define MOTOR_NUM            16

/* 系统运行状态 */
#define SYSTEM_STOPPED 0
#define SYSTEM_RUNNING 1

/* 异常标志位 */
#define NO_EXCEPTION   0
 
/*******************************************************************************************************************/
/* Type Defines                                                                                                    */
/*******************************************************************************************************************/
 
typedef struct _TickTimer
{
    uint32_t ticks_last;
    uint32_t ticks_now;
} TickTimer;

typedef struct _Step_motor
{
	uint8_t spix;         		//电机的选择  

	uint32_t mres;              //细分微步  page:46
	uint32_t dir;               //方向      page:27
	uint32_t current;           //电流      page:33
	uint32_t current_delay;     //电流延迟

	uint32_t mode;              //0：位置模式 1：速度模式到正VMAX 2：速度模式至负VMAX 3：保持模式       page:35
	uint32_t xactual;           //实际电机位置(signed)
	uint32_t vactual;           //斜坡发生器产生的实际电机速度(signed)
	uint32_t vstart;            //电机启动速度(unsigned)
	uint32_t a1;                //VSTART 和 V1 之间的加速度(无符号)         
	uint32_t v1;                //第一加速/减速阶段阈值速度(无符号)
	uint32_t amax;              //V1 和 VMAX 之间的加速度(无符号)
	uint32_t vmax; 	            //运动斜坡目标速度(位置模式确保VMAX ≥ VSTART ) (无符号)     
	uint32_t dmax; 	            //VMAX 和 V1 之间的减速度(无符号)
	uint32_t d1; 	            //V1 和 VSTOP 之间的减速度(无符号)
	uint32_t vstop;             //电机停止速度(无符号)
	
	uint8_t has5048bx;
	uint8_t limitx;
	uint32_t vmax_default; 		//运动斜坡目标速度的默认值
	
}Step_motor;
	
/*******************************************************************************************************************/
/* Global variables declaration                                                                                    */
/*******************************************************************************************************************/
extern Step_motor step_motor[5];
//extern uint32_t taskReadyFlags; // 就绪任务标志

/*******************************************************************************************************************/
/* Tasks Declaration                                                                                               */
/*******************************************************************************************************************/

//void SystemTasksInit(void);
void TaskScheduler(void);
//void ExceptionHandler(void);

static uint8_t *ExecutePCCommand(uint8_t *cmd, uint8_t len);

uint8_t IsTimeOut(TickTimer *timer, uint32_t time_out);
void ResetTickTimer(TickTimer *timer);

#endif

