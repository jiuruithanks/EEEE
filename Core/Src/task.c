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

/* ��ʱ�� */
//static TickTimer twinkleLED_Timer = {0, 0}; // ����LED��˸
//static TickTimer errorLED_Timer   = {0, 0}; // ����errorLED��˸

/* ϵͳ����״̬��� */
uint32_t errorCode[MAX_TASK_NUM] = {0};            // ���ڼ�¼ÿ��������쳣��־
uint8_t systemStatus             = SYSTEM_RUNNING; // ϵͳ����״̬
uint32_t taskReadyFlags          = 0;     // ���о��������־

/* �˶�������� */
Step_motor step_motor[5];	//����������ݽṹ


void TaskScheduler(void)
{
    /* ���ڽ�������PC������ */
    uint8_t cmd[20] = {0};
    uint8_t len     = 0;

	cmd[0] = PC_WRITE_EEPROM;
    if (SYSTEM_RUNNING == systemStatus)
    {
        /* ����PC���е��ԡ���ӡ��־��� */
		len = UART3_Rx_cnt;
        if (len)
        {
            ExecutePCCommand(cmd, len);
			UART3_Rx_cnt = 0;
        }
    }
}

/**
 * @brief ִ������PCͨ�����ڷ��͹�����ָ�
 *
 * @param cmd ָ������
 * @param len ָ���
 * @return uint8_t*
 */
static uint8_t *ExecutePCCommand(uint8_t *cmd, uint8_t len)
{
	 /* ��ʱ���� */
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
 * @brief �ж����ھ�����һ�����øö�ʱ����ʱ�䳤���Ƿ��Ѿ��ﵽtime_out���롣
 *
 * @param timer ��ʱ���ṹ��ָ��
 * @param time_out ��ʱʱ�䣬��λ��ms��
 * @return uint8_t ����Ϊ1��ʾ��ʱ������Ϊ0��ʾδ��ʱ��
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
 * @brief ���ö�ʱ��
 *
 * @param timer ��ʱ���ṹ��ָ��
 */
void ResetTickTimer(TickTimer *timer)
{
    timer->ticks_last = HAL_GetTick();
}

///**
// * @brief �����쳣�������һ�δ������쳣�����쳣��Ϣ��¼����־�С�
// *
// * @param task_name �쳣���ĸ����񴥷���
// * @param exception �쳣�����־λ
// */
//void RaiseException(uint32_t task_name, uint32_t exception)
//{
//    if ((errorCode[task_name] & exception) == 0)
//    {
//        /* ˵��������ĸô����һ�δ��� */
//        errorCode[task_name] |= exception;
//        LogError(task_name, exception);
//    }
//}

///**
// * @brief ����쳣
// *
// * @param task_name �쳣���ĸ����񴥷���
// * @param exception �쳣�����־λ
// */
//void ClearException(uint32_t task_name, uint32_t exception)
//{
//    errorCode[task_name] &= ~exception;
//}

///**
// * @brief �������趨Ϊ����״̬
// *
// * @param task_name ��������
// */
//void SetTaskReady(uint32_t task_name)
//{
//    taskReadyFlags |= (1 << task_name);
//}

///**
// * @brief �������ľ���״̬
// *
// * @param task_name ��������
// */
//void SuspendTask(uint32_t task_name)
//{
//    taskReadyFlags &= ~(1 << task_name);
//}

///**
// * @brief �жϸ������Ƿ��ھ���״̬
// *
// * @param task_name ��������
// * @return uint8_t ����ֵΪ1��ʾ�������������ֵΪ0��ʾ����δ�������޷�����
// */
//uint8_t IsTaskReady(uint32_t task_name)
//{
//    return ((SYSTEM_RUNNING == systemStatus) && (taskReadyFlags & (1 << task_name)));
//}


