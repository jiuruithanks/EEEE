/**
 * @file task.c
 * @author Zhao Haofei
 * @brief task on STM32F407VET6.
 * @version 1.0
 * @date 2023-06-14
 *
 * @copyright Copyright (c) 2023
 */
#include "main.h"
#include "task.h"
#include "24cxx.h"
#include "usart.h"


/*******************************************************************************************************************/
/* Global Variables                                                                                                */
/*******************************************************************************************************************/

/* 计时器 */
//static TickTimer twinkleLED_Timer = {0, 0}; // 用于LED闪烁
//static TickTimer errorLED_Timer   = {0, 0}; // 用于errorLED闪烁

/* 系统运行状态相关 */
uint32_t errorCode[MAX_TASK_NUM] = {0};            // 用于记录每个任务的异常标志
uint8_t systemStatus             = SYSTEM_RUNNING; // 系统运行状态
uint32_t taskReadyFlags          = 0;     // 所有就绪任务标志

/* 运动控制相关 */
Step_motor step_motor[5];	//步进电机数据结构


void TaskScheduler(void)
{
    /* 用于接收来自PC的命令 */
    uint8_t cmd[20] = {0};
    uint8_t len     = 0;

	cmd[0] = PC_WRITE_EEPROM;
    if (SYSTEM_RUNNING == systemStatus)
    {
        /* 用于PC进行调试、打印日志相关 */
		len = UART3_Rx_cnt;
        if (len)
        {
            ExecutePCCommand(cmd, len);
			UART3_Rx_cnt = 0;
        }
    }
}

/**
 * @brief 执行来自PC通过串口发送过来的指令。
 *
 * @param cmd 指令数据
 * @param len 指令长度
 * @return uint8_t*
 */
static uint8_t *ExecutePCCommand(uint8_t *cmd, uint8_t len)
{
	 /* 临时变量 */
    uint8_t *res   = 0;
//    static uint8_t a[16] = {0};
//    uint8_t status = 0;
    uint32_t temp = 0;
	
	switch (cmd[0])
	{
		case PC_WRITE_EEPROM:
//			temp = (cmd[1] << 16) | (cmd[2] << 8) | cmd[3]; // cmd[1]: Block; cmd[2]: Page
			temp = cmd[0];
			at24cxx_write(temp, cmd + 1, len - 1);
		break;
		default:
			HAL_UART_Transmit(&huart3, (uint8_t *)"Invalid cmd\r\n", 13, 50);
		break;
	}
	return res;
}

/**
 * @brief 判断现在距离上一次重置该定时器的时间长度是否已经达到time_out毫秒。
 *
 * @param timer 计时器结构体指针
 * @param time_out 超时时间，单位是ms。
 * @return uint8_t 返回为1表示超时，返回为0表示未超时。
 */
uint8_t IsTimeOut(TickTimer *timer, uint32_t time_out)
{
    timer->ticks_now = HAL_GetTick();
    if (timer->ticks_last + time_out <= timer->ticks_now)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief 重置定时器
 *
 * @param timer 计时器结构体指针
 */
void ResetTickTimer(TickTimer *timer)
{
    timer->ticks_last = HAL_GetTick();
}

///**
// * @brief 触发异常。如果第一次触发该异常，则将异常信息记录在日志中。
// *
// * @param task_name 异常是哪个任务触发的
// * @param exception 异常代码标志位
// */
//void RaiseException(uint32_t task_name, uint32_t exception)
//{
//    if ((errorCode[task_name] & exception) == 0)
//    {
//        /* 说明该任务的该错误第一次触发 */
//        errorCode[task_name] |= exception;
//        LogError(task_name, exception);
//    }
//}

///**
// * @brief 清除异常
// *
// * @param task_name 异常是哪个任务触发的
// * @param exception 异常代码标志位
// */
//void ClearException(uint32_t task_name, uint32_t exception)
//{
//    errorCode[task_name] &= ~exception;
//}

///**
// * @brief 将任务设定为就绪状态
// *
// * @param task_name 任务名称
// */
//void SetTaskReady(uint32_t task_name)
//{
//    taskReadyFlags |= (1 << task_name);
//}

///**
// * @brief 清除任务的就绪状态
// *
// * @param task_name 任务名称
// */
//void SuspendTask(uint32_t task_name)
//{
//    taskReadyFlags &= ~(1 << task_name);
//}

///**
// * @brief 判断该任务是否处于就绪状态
// *
// * @param task_name 任务名称
// * @return uint8_t 返回值为1表示任务就绪，返回值为0表示任务未就绪，无法运行
// */
//uint8_t IsTaskReady(uint32_t task_name)
//{
//    return ((SYSTEM_RUNNING == systemStatus) && (taskReadyFlags & (1 << task_name)));
//}


