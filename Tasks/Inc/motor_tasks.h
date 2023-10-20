#ifndef __MOTOR_TASKS_H
#define __MOTOR_TASKS_H

#include "task.h"
#include "pca9539.h"
#include "tmc5160.h"
#include "bsp_dcmotor.h"
#include "bsp.h"

/****************************************************** �궨�� ***********************************************************************/
#define LIMIT_LEFT  98
#define LIMIT_RIGHT 157

#define INJECT      0    //1:ע��� 0:ERCP��е����
/****************************************************** �ṹ�� ***********************************************************************/
typedef struct key_val
{
	uint8_t keyVal[16];
	uint8_t keyValPre[16];
	uint8_t key_action[16];
}KEY_VAL;

typedef struct POSITION
{
	uint16_t device1_min_pos;	//Uartmotor3_mintarget
	uint16_t device1_max_pos;	//Uartmotor3_maxtarget
	uint16_t device1_real_pos;	//ʵ��λ��
	uint16_t device1_tar_pos;	//Ŀ��λ��
}POSITION_HandleTypeDef;


/****************************************************** �ṩ�ⲿ���� ***********************************************************************/
extern KEY_VAL key_val;
extern POSITION_HandleTypeDef POSITION_DEVICE;
extern float g_increment;
extern float device1_tar_pos; 

extern TickTimer bileDuctTimer;

/****************************************************** ERCP��е������� ***********************************************************************/
void Change_Device(SPI_CS_TypeDef *CS, PCA9555_HandleTypeDef *key);
void Updata_Current_device(PCA9555_HandleTypeDef *hdev_key,PCA9555_HandleTypeDef *hdev_led ,DEVICE_GW_Typedef *device_actionx);
void Update_step_motor(PCA9555_HandleTypeDef *hdev_key, DEVICE_GW_Typedef *gwx, DEVICE_GW_Typedef *devicex, DEVICE_GW_Typedef *device_and_gw_actionx);



void Update_Motor_Status(PCA9555_HandleTypeDef *hdev_key, DEVICE_GW_Typedef *gwx, DEVICE_GW_Typedef *devicex, DEVICE_GW_Typedef *device_actionx, DEVICE_GW_Typedef *device_and_gw_movex);

/*********************** ���ڵ���˶�������غ��� ********************************/
void Update_uartmotor_target(uint16_t tar_pos);



/*********************** ע����˶�������غ��� ********************************/
// void Panel_Step(uint8_t angle_val, SPI_CS_TypeDef *CS);
void key_step_inject(void);
void key_dc_inject(void);
void panel_step_inject(DEVICE_GW_Typedef *inject_action, SPI_CS_TypeDef *CS);
void panel_dc_inject(DEVICE_GW_Typedef *inject_action);
void injection_control(DEVICE_GW_Typedef *encoder_select);
void step_test(SPI_CS_TypeDef *CS);


/*********************** ֱ������˶�������غ��� ********************************/
void Updata_dcmtor(PCA9555_HandleTypeDef *hdev_key);

#endif /* MOTOR_TASKS_H */

