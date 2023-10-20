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

/* �����������͸�STM32������������ԡ���ӡ��־�� */
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

/* ϵͳ����״̬ */
#define SYSTEM_STOPPED 0
#define SYSTEM_RUNNING 1

/* �쳣��־λ */
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
	uint8_t spix;         		//�����ѡ��  

	uint32_t mres;              //ϸ��΢��  page:46
	uint32_t dir;               //����      page:27
	uint32_t current;           //����      page:33
	uint32_t current_delay;     //�����ӳ�

	uint32_t mode;              //0��λ��ģʽ 1���ٶ�ģʽ����VMAX 2���ٶ�ģʽ����VMAX 3������ģʽ       page:35
	uint32_t xactual;           //ʵ�ʵ��λ��(signed)
	uint32_t vactual;           //б�·�����������ʵ�ʵ���ٶ�(signed)
	uint32_t vstart;            //��������ٶ�(unsigned)
	uint32_t a1;                //VSTART �� V1 ֮��ļ��ٶ�(�޷���)         
	uint32_t v1;                //��һ����/���ٽ׶���ֵ�ٶ�(�޷���)
	uint32_t amax;              //V1 �� VMAX ֮��ļ��ٶ�(�޷���)
	uint32_t vmax; 	            //�˶�б��Ŀ���ٶ�(λ��ģʽȷ��VMAX �� VSTART ) (�޷���)     
	uint32_t dmax; 	            //VMAX �� V1 ֮��ļ��ٶ�(�޷���)
	uint32_t d1; 	            //V1 �� VSTOP ֮��ļ��ٶ�(�޷���)
	uint32_t vstop;             //���ֹͣ�ٶ�(�޷���)
	
	uint8_t has5048bx;
	uint8_t limitx;
	uint32_t vmax_default; 		//�˶�б��Ŀ���ٶȵ�Ĭ��ֵ
	
}Step_motor;
	
/*******************************************************************************************************************/
/* Global variables declaration                                                                                    */
/*******************************************************************************************************************/
extern Step_motor step_motor[5];
//extern uint32_t taskReadyFlags; // ���������־

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

