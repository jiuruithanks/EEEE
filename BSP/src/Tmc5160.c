#include <math.h>
#include <stdio.h>
#include "main.h"
#include "tmc5160.h"
#include "bsp_pid.h"
#include "motor_tasks.h"
#include "delay.h"

/* Private variables ---------------------------------------------------------*/

unsigned 	char DirFlag=0;
char Spi_Status;					//PAGE23:SPI����״̬λ
unsigned 	long register_value = 0;
long ReadPosition;
long PositionData = 0;			//����Ŀ��λ��ֵ

uint16_t pos_min	= 49;		//����λ1
uint16_t pos_max 	= 319;		//����λ4
uint16_t pos_mask 	= 359;   	//359


uint8_t is_ok_to_run = 0;	//�жϵ���Ƿ���Ҫ�˶�
HAL_StatusTypeDef status;	//״̬


uint16_t Vstart = 100;
uint16_t A1 = 10;
uint16_t V1 = 5000;
uint16_t AMAX = 1000;
uint16_t VMAX = 10000;
uint16_t DMAX = 1000;
uint16_t D1 = 100;

uint8_t Step_Motor_Flag[8] = {0};
uint8_t Step_Motor_Flag_pre[8] = {0};

uint8_t g_switch_head_status;
uint8_t g_switch_head_status_pre;
uint8_t g_switch_back_status;
uint8_t g_switch_back_status_pre;
uint8_t g_delay_flag;

/****************************************** �ṹ�� ***********************************************/
MOTOR_LIMIT_TypeDef STEP_MOTOR4_LIMIT=
{
	LIMIT4_GPIO_Port,
	LIMIT4_Pin,
	LIMIT5_GPIO_Port,
	LIMIT5_Pin,
};

MOTOR_LIMIT_TypeDef STEP_MOTOR5_LIMIT=
{
	LIMIT7_GPIO_Port,
	LIMIT7_Pin,
	LIMIT6_GPIO_Port,
	LIMIT6_Pin,
};



//////*�������˵��*////////////////////////////////
/**
 * SPICS1 ��˿����
 * SPICS2 ��е����
 * SPICS3 ��е�л���ת
 * SPICS4 ȡʯ����
 * SPICS5 ����֧��
 * 
 * **/
#if INJECT
SPI_CS_TypeDef SPICS1={
SPI1_CSN1_GPIO_Port,
SPI1_CSN1_Pin,
TMC_EN1_GPIO_Port,
TMC_EN1_Pin,
0x000100C3, //ϸ�� mres  BIT27-BIT24  CHOPCONF 
0x00000004, //dir
0x000200A03, //current 0x00020F03	test 0x00021F03		IRUN 0x08  IHOLD 0x03
0x0000000A, //current_delay
0x00000001, //mode
0,			//xactual
0,		    //vactual
1000,		//vstart
50000,	 	//���ٶ�a1
50000, 		//��һ�׶��ٶȷ�ֵv1
50000,		//V1��VMAX����ٶ�amax
300000,		//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4  ===��100000*5.1 �������
50000,		//dmax
50000,		//d1
10,			//vstop
0,
0,
300000,		//Ŀ���ٶȵ�Ĭ��ֵ	old 510000
};

SPI_CS_TypeDef SPICS2={
SPI1_CSN2_GPIO_Port,
SPI1_CSN2_Pin,
TMC_EN2_GPIO_Port,
TMC_EN2_Pin,
0x000100C3, //ϸ�� mres  BIT27-BIT24  
0x00000004, //dir
0x00020F03, //current
0x0000000A, //current_delay
0x00000001, //mode
0,			//xactual
0,		   	//vactual
1000,		//vstart
50000,		//���ٶ�a1
50000, 		//��һ�׶��ٶȷ�ֵv1
50000,		//V1��VMAX����ٶ�amax
300000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4  ===��100000*3.7 �������
50000,		//dmax
50000,		//d1
10	,		//vstop
0,
0,
300000,		//Ŀ���ٶȵ�Ĭ��ֵ	old 370000	The same value of vmax and vmax_default
};

SPI_CS_TypeDef SPICS3={
SPI1_CSN3_GPIO_Port,
SPI1_CSN3_Pin,
TMC_EN3_GPIO_Port,
TMC_EN3_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020A03, //current	4+5+5   0-19
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		   	//vactual
1000,		//vstart
50000,		//���ٶ�a1
50000, 		//��һ�׶��ٶȷ�ֵv1
50000,		//V1��VMAX����ٶ�amax
300000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
50000,		//dmax
50000,		//d1
10,			//vstop
&as5048b10,
0,
300000,		//Default value for the target speed Ŀ���ٶȵ�Ĭ��ֵ	old 200000
};

SPI_CS_TypeDef SPICS4={
SPI1_CSN4_GPIO_Port,
SPI1_CSN4_Pin,
TMC_EN4_GPIO_Port,
TMC_EN4_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020A03,	//current
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		  	//vactual
1000,		//vstart
1000,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
200000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
700,		//dmax
1400,		//d1
10,			//vstop
0,
&STEP_MOTOR4_LIMIT,
200000,		//Default value for the target speed	old 200000
};

SPI_CS_TypeDef SPICS5={
SPI1_CSN5_GPIO_Port,
SPI1_CSN5_Pin,
TMC_EN5_GPIO_Port,
TMC_EN5_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020A03,	//current
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		   	//vactual
100,		//vstart
100,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
200000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
700,		//dmax
1400,		//d1
10,			//vstop
0,
&STEP_MOTOR5_LIMIT,
200000,		//Default value for the target speed  Ŀ���ٶȵ�Ĭ��ֵ	old 200000
};

#else
SPI_CS_TypeDef SPICS1={
SPI1_CSN1_GPIO_Port,
SPI1_CSN1_Pin,
TMC_EN1_GPIO_Port,
TMC_EN1_Pin,
0x000100C3, //ϸ�� mres  BIT27-BIT24  
0x00000004, //dir
0x000200F03, // 0x00020F03, //current 0x00020F03	test 0x00021F03		IRUN 0x08  IHOLD 0x03
0x0000000A, //current_delay
0x00000001, //mode
0,			//xactual
0,		    //vactual
100,		//vstart
100,	 	//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
370000, 	//510000,		//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4  ===��100000*5.1 �������
700,		//dmax
1400,		//d1
10,			//vstop
0,
0,
370000,     //510000,		//Ŀ���ٶȵ�Ĭ��ֵ	old 510000
};

SPI_CS_TypeDef SPICS2={
SPI1_CSN2_GPIO_Port,
SPI1_CSN2_Pin,
TMC_EN2_GPIO_Port,
TMC_EN2_Pin,
0x000100C3, //ϸ�� mres  BIT27-BIT24  
0x00000004, //dir
0x00020F03, //current
0x0000000A, //current_delay
0x00000001, //mode
0,			//xactual
0,		   	//vactual
1000,		//vstart
1000,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
268300, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4  ===��100000*3.7 �������
700	,		//dmax
1400,		//d1
10	,		//vstop
0,
0,
268300,		//Ŀ���ٶȵ�Ĭ��ֵ	old 370000	The same value of vmax and vmax_default
};

SPI_CS_TypeDef SPICS3={
SPI1_CSN3_GPIO_Port,
SPI1_CSN3_Pin,
TMC_EN3_GPIO_Port,
TMC_EN3_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020A03, //current	4+5+5   0-19
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		   	//vactual
1000,		//vstart
1000,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
10000,		//V1��VMAX����ٶ�amax
300000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
700,		//dmax
1400,		//d1
10,			//vstop
&as5048b10,
0,
300000,		//Default value for the target speed Ŀ���ٶȵ�Ĭ��ֵ	old 200000
};

SPI_CS_TypeDef SPICS4={
SPI1_CSN4_GPIO_Port,
SPI1_CSN4_Pin,
TMC_EN4_GPIO_Port,
TMC_EN4_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020A03,	//current
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		  	//vactual
1000,		//vstart
1000,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
300000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
700,		//dmax
1400,		//d1
10,			//vstop
0,
&STEP_MOTOR4_LIMIT,
300000,		//Default value for the target speed	old 200000
};

SPI_CS_TypeDef SPICS5={
SPI1_CSN5_GPIO_Port,
SPI1_CSN5_Pin,
TMC_EN5_GPIO_Port,
TMC_EN5_Pin,
0x000100C3,	//ϸ�� mres  BIT27-BIT24  
0x00000004,	//dir
0x00020D03,	//current
0x0000000A,	//current_delay
0x00000001,	//mode
0,			//xactual
0,		   	//vactual
100,		//vstart
1000,		//���ٶ�a1
20000, 		//��һ�׶��ٶȷ�ֵv1
1000,		//V1��VMAX����ٶ�amax
200000, 	//358400, 	  //Ŀ���ٶ�vmax   200*256*5*1.4
700,		//dmax
1400,		//d1
10,			//vstop
&as5048b11,
&STEP_MOTOR5_LIMIT,
200000,		//Default value for the target speed  Ŀ���ٶȵ�Ĭ��ֵ	old 200000
};
#endif

/* USER CODE BEGIN PV */
#define	SPI_TIMEOUT_VALUE	500				//�����дSPI���ݳ�ʱʱ�䶼Ϊ500	

#define	SPI_TIMEOUT_VALUE1	500				//�����дSPI���ݳ�ʱʱ�䶼Ϊ500	
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
void Spi_Cs_init(void)
{
}
/* USER CODE END 0 */

/**
  ******** 
	��ʼ��TMC5160��ؼĴ�������
  ******** 
*/
void Tmc5160Initial(SPI_CS_TypeDef* CS)
{															//IHOLDDELAY ���õ����⵽��ֹ( stst = 1 )�� TPOWERDOWN ֮�󣬽��������̵�ʱ����������
	sendData_SPI1(REG_CHOPCONF,CS->mres, CS); 					//PAGE46:CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadcycle)
	sendData_SPI1(REG_IHOLD_IRUN,CS->current, CS); 			//PAGE33:IHOLDDELAY+IRUN+IHOLD 4+5+5λ  IRUN=0x08 ������е���  IHOLD=0x03  ��ֹ״̬�µ������  
	sendData_SPI1(REG_TPOWERDOWN,CS->current_delay, CS); //PAGE33:IHOLDDELAY+IRUN+IHOLD 4+5+5λ  IRUN=0x08  IHOLD=0x03 
	sendData_SPI1(REG_GCONF,CS->dir, CS);						//PAGE27:EN_PWM_MODE=1��ʹ��STEALTHCHOP, shaft =1 �������
	sendData_SPI1(REG_VSTART,CS->vstart, CS);   			//�����ٶ�   0 ~2^18-1

	sendData_SPI1(REG_A1,CS->a1, CS);     				//PAGE35:A1=1000 ��һ�׶μ��ٶ�		2^16 -1
	sendData_SPI1(REG_V1,CS->v1, CS);     				//PAGE35:V1=50000���ٶȷ�ֵ�ٶ�V1	2^20 -1
	sendData_SPI1(REG_AMAX,CS->amax, CS);     			//PAGE35:AMAX=500����V1�ļ��ٶ�    �ٶ�ģʽ�ļ��ٶ�ֵ   2000    2^16 -1
	sendData_SPI1(REG_VMAX,CS->vmax, CS);     			//PAGE35:VMAX�ٶ�ģʽ�µ�Ŀ���ٶ�                              2^23-512     
	sendData_SPI1(REG_DMAX,CS->dmax, CS);				//PAGE35:DMAX=700����V1�ļ��ٶ� 	VMAX��V1֮��ļ��ٶ�	    2^16 -1
	sendData_SPI1(REG_D1,CS->d1, CS);     				//PAGE35:D1=1400С��V1�ļ��ٶ�     ��Ҫ��λ��ģʽ����Ϊ 0����ʹ V1 = 0�� 
	sendData_SPI1(REG_VSTOP,CS->vstop,  CS);     		//PAGE35:VSTOP=10ֹͣ�ٶȣ��ӽ���0		
	sendData_SPI1(REG_MODE,CS->mode,CS);				//MODE	rampmode = 0:λ��ģʽ	1:�ٶ�ģʽ����VMAX	2���ٶ�ģʽ����VMAX	3������ģʽ

}

/**
 * @brief SPI���ֽڽ���
 * 
 * @param Send_Data 
 * @return char 
 */
static char SPI_ReceiveByte(char Send_Data)
{ 
	unsigned char	cmd[2];
	
	cmd[0] = Send_Data;
	if(HAL_SPI_Transmit(&hspi1, cmd, 1, SPI_TIMEOUT_VALUE) == HAL_OK)
	{		//�������ݳɹ���������
		if(HAL_SPI_Receive(&hspi1, cmd, 1, SPI_TIMEOUT_VALUE) == HAL_OK)
		{  
			return cmd[0];	//�����ݳɹ�����������
		}
		else return 0x5a;	//������ʧ�ܣ�����0x5a
	}
	else return 0x5a;			//��������ʧ�ܣ�����0x5a
}

/**
 * @brief ����SPI����
 * 
 * @param address �Ĵ�������ַ
 * @param datagram 32λ������
 * @param CS Ƭѡ
 */
void sendData_SPI1(unsigned long address,long datagram,SPI_CS_TypeDef* CS)
{	
	unsigned char	cmd[5];
	HAL_GPIO_WritePin(CS->GPIOx, CS->Pin, GPIO_PIN_RESET);  	//SPI_CSƬѡ����
	cmd[0]=address;
	cmd[1]=(uint8_t)(datagram >> 24);
	cmd[2]=(uint8_t)(datagram >> 16);
	cmd[3]=(uint8_t)(datagram >> 8);
	cmd[4]=(uint8_t)(datagram);
	if(HAL_SPI_TransmitReceive(&hspi1, cmd, cmd, 5, SPI_TIMEOUT_VALUE) == HAL_OK)
	{
	}
	else;
	HAL_GPIO_WritePin(CS->GPIOx, CS->Pin, GPIO_PIN_SET);   	//SPI_CSƬѡ����
}

/**
 * @brief ��ȡSPI����
 * 
 * @param address 
 * @param CS 
 * @return long 
 */
long ReadData_SPI1(char address, SPI_CS_TypeDef* CS)
{
	long datagram;
	unsigned char cmd[5],ReadData5160[5];
	
	HAL_GPIO_WritePin(CS->GPIOx, CS->Pin, GPIO_PIN_RESET); 	//SPI_CSƬѡ����
	
	cmd[0]=address;																											
	if(HAL_SPI_TransmitReceive(&hspi1, cmd, ReadData5160, 5, SPI_TIMEOUT_VALUE) == HAL_OK)
	{
		datagram = (ReadData5160[1] << 24) | (ReadData5160[2] << 16) | (ReadData5160[3] << 8) | ReadData5160[4];			
	}
	else	datagram = 0x5a5a5a5a;
	Spi_Status = ReadData5160[0];																					//PAGE23:SPI����״̬	
	HAL_GPIO_WritePin(CS->GPIOx, CS->Pin, GPIO_PIN_SET); 	//SPI_CSƬѡ���� 		//SPI_CSƬѡ����
	
	CS->vactual = datagram;
	return datagram;
}

/**
 * @brief ���ٶ�ʽ�������в������
 * 
 */
void Run_All_Step_Motor(void)		
{
	Run_STEP_Motor_SPEED(&SPICS1, 1);
	Run_STEP_Motor_SPEED(&SPICS2, 1);
	Run_STEP_Motor_SPEED(&SPICS3, 1);
	Run_STEP_Motor_SPEED(&SPICS4, 1);
	Run_STEP_Motor_SPEED(&SPICS5, 1);	
}

/**
 * @brief ʧ�����в������
 * 
 */
void Dis_All_Step_Driver(void)
{
	HAL_GPIO_WritePin(TMC_EN1_GPIO_Port,TMC_EN1_Pin,GPIO_PIN_SET);
  	HAL_GPIO_WritePin(TMC_EN2_GPIO_Port,TMC_EN2_Pin,GPIO_PIN_SET);
  	HAL_GPIO_WritePin(TMC_EN3_GPIO_Port,TMC_EN3_Pin,GPIO_PIN_SET);
 	HAL_GPIO_WritePin(TMC_EN4_GPIO_Port,TMC_EN4_Pin,GPIO_PIN_SET);
 	HAL_GPIO_WritePin(TMC_EN5_GPIO_Port,TMC_EN5_Pin,GPIO_PIN_SET);
}

/**
 * @brief ʹ�����в������
 * 
 */
void EN_All_Step_Driver(void)
{
  	HAL_GPIO_WritePin(TMC_EN1_GPIO_Port,TMC_EN1_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(TMC_EN2_GPIO_Port,TMC_EN2_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(TMC_EN3_GPIO_Port,TMC_EN3_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(TMC_EN4_GPIO_Port,TMC_EN4_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(TMC_EN5_GPIO_Port,TMC_EN5_Pin,GPIO_PIN_RESET);
}

/**
 * @brief �������λ��ʽ�˶�
 * 
 * @param CS Ƭѡ
 * @param tar_pos Ŀ��λ��
 */
void Run_STEP_Motor_POS(SPI_CS_TypeDef *CS, uint8_t tar_pos)	
{
	SPI_CS_TypeDef *p = CS;

	uint8_t is_run_to_increase_pos = 0;

	uint16_t as5048b_temp_angle = 0;
	uint8_t pid_out = 0;
	
	int distance_from_min_to_pos = 0;
	int distance_from_pos_to_max = 0;
	int cross_zone = 0;
	int is_pos_corss_min = 0;
	int is_pos_cross_max = 0;
	float tar_angle = 0;
	float cur_angle = 0;
	float re_cur_angle = 0;	//��ӳ���ĽǶ�  The Angle after remapping
	
	is_ok_to_run = 1;

	tar_angle = change_target_angle(tar_pos);
	
	
	AS5048B_Read(p->has5048b, p->has5048b->ADDRESS, &as5048b_temp_angle);
	cur_angle = p->has5048b->angle_f;
	//ƫ��90��
	cur_angle = (uint16_t)(360.0f - cur_angle + 90.0f) % (pos_mask + 1);	//ת����������ת
	

	distance_from_min_to_pos = (int)((pos_min + pos_mask + 1 - (int)cur_angle) % (pos_mask + 1));
	distance_from_pos_to_max = (((int)cur_angle + pos_mask + 1 - pos_max) % (pos_mask + 1));
	cross_zone = (pos_min - pos_max) / 3;

	is_pos_corss_min = (distance_from_min_to_pos > 0) && (distance_from_min_to_pos < cross_zone);
	is_pos_cross_max = (distance_from_pos_to_max > 0) && (distance_from_pos_to_max < cross_zone);
	
	//Remapping of angles 	0:49  90:139  180:229  270:319
	re_cur_angle = cur_angle - pos_min;	


	pid_out = abs(PID_send(tar_angle, (int)re_cur_angle));
	if(pid_out > 50)
	pid_out = 50;
	else if(pid_out <10)
	pid_out = 10;
	else 
	pid_out = pid_out;
			
	is_run_to_increase_pos = re_cur_angle < tar_angle ? 1 : 0;
	
	if(is_pos_corss_min && (!is_run_to_increase_pos))
	is_ok_to_run = 0;
	else if (is_pos_cross_max && is_run_to_increase_pos)
	is_ok_to_run = 0;
	else if (fabs(re_cur_angle - tar_angle) < 0.3f)
	is_ok_to_run = 0;
	else if (cur_angle == 0)
	is_ok_to_run = 0;


	if(re_cur_angle < tar_angle - 0.3f)
	{	
		sendData_SPI1(REG_GCONF, 0x00000010, p);
	}
	else if (re_cur_angle > tar_angle + 0.3f)
	{
		sendData_SPI1(REG_GCONF, 0x00000000, p);
	}
	else 
	is_ok_to_run = 0;
	
	sendData_SPI1(REG_MODE, CS->mode, CS);			//MODE	
	sendData_SPI1(REG_CHOPCONF, CS->mres, CS); 		//PAGE46:CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadcycle)
	sendData_SPI1(REG_IHOLD_IRUN, CS->current, CS); 	//PAGE33:IHOLDDELAY+IRUN+IHOLD 4+5+5λ    IRUN=0x08   IHOLD=0x03   
	sendData_SPI1(REG_VMAX, (p->vmax) * pid_out / 50, p); 
	//PAGE35:VMAX�ٶ�ģʽ�µ�Ŀ���ٶ�                               2^23-512 

	sendData_SPI1(REG_XACTUAL, p->xactual, p);

	if(is_ok_to_run == 1)
	{
		HAL_GPIO_WritePin(p->EN_GPIOx, p->EN_Pin, GPIO_PIN_RESET);
	}	
	else if(is_ok_to_run == 0)
	{
		HAL_GPIO_WritePin(p->EN_GPIOx, p->EN_Pin, GPIO_PIN_SET);
	}
}


/**
 * @brief ��������ٶ�ʽ�˶�
 * 
 * @param CS Ƭѡ
 * @param dir ����
 */


uint8_t g_head_stop;
uint8_t g_back_stop;
long g_temp;
void Run_STEP_Motor_SPEED(SPI_CS_TypeDef *CS, int dir)	
{
	SPI_CS_TypeDef *p = CS;
	int dir_temp = 0;
	dir_temp = dir;
	p->dir = dir;
	uint8_t is_ok_to_run = 1;
	

	if (dir_temp == 1)	
	sendData_SPI1(REG_GCONF, 0x00000000, p);			//PAGE27:shaft =0 ���CCW			
	else 
	sendData_SPI1(REG_GCONF, 0x00000010, p);			//PAGE27:shaft =1 ���CCW	
	
	sendData_SPI1(REG_MODE, CS->mode, CS);			//MODE	
	sendData_SPI1(REG_CHOPCONF, CS->mres, CS); 		//PAGE46:CHOPCONF: TOFF=3, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadcycle)
	sendData_SPI1(REG_IHOLD_IRUN, CS->current, CS); 	//PAGE33:IHOLDDELAY+IRUN+IHOLD 4+5+5λ    IRUN=0x08   IHOLD=0x03   
	sendData_SPI1(REG_VMAX, p->vmax, p);
	//�˶���������ز���
	sendData_SPI1(REG_A1, p->a1, p);
	sendData_SPI1(REG_V1, p->v1, p);
	sendData_SPI1(REG_AMAX, p->amax, p);
	sendData_SPI1(REG_VMAX, p->vmax, p);     		//PAGE35:VMAX�ٶ�ģʽ�µ�Ŀ���ٶ�            2^23-512 
	sendData_SPI1(REG_DMAX, p->dmax, p);
	sendData_SPI1(REG_D1, p->d1, p);
	sendData_SPI1(REG_VSTOP, p->vstop, p);

	
	g_temp = ReadData_SPI1(0xA2, p);
	//stealthchop���
// 	sendData_SPI1(REG_IHOLD_IRUN, 0x0006100A, p);
// 	sendData_SPI1(REG_TPOWERDOWN, 0x0000000A, p);
//	sendData_SPI1(REG_GCONF, 0x00000004, p);	//ʹ��PWMģʽ
// 	sendData_SPI1(REG_TPWMTHRS, 0x000001F4, p);	// TPWM_THRS=500 ��Ӧ�л��ٶ� 35000 = ca. 30RPM
// 	sendData_SPI1(REG_PWMCONF, 0x0000000A, p);	//PWM����ģʽ

	g_switch_head_status = HAL_GPIO_ReadPin(p->pos_limit->GPIOx_head, p->pos_limit->Pin_head);
	g_switch_back_status = HAL_GPIO_ReadPin(p->pos_limit->GPIOx_back, p->pos_limit->Pin_back);


	if((p == &SPICS4) || (p == &SPICS5))
	{
		
//	g_switch_head_status = HAL_GPIO_ReadPin(p->pos_limit->GPIOx_head, p->pos_limit->Pin_head);
//	g_switch_back_status = HAL_GPIO_ReadPin(p->pos_limit->GPIOx_back, p->pos_limit->Pin_back);
		
		if ((dir_temp == 0) && ((g_switch_head_status_pre == 1) && (g_switch_head_status == 0)))
		{
			g_head_stop = 1;
		}
		else if ((dir_temp == 1) && (g_switch_back_status_pre == 1) && (g_switch_back_status == 0))
		{
			g_back_stop = 1;
		}
		
		if (dir_temp == 0)
			g_back_stop = 0;
		else 
			g_head_stop = 0;

		g_switch_head_status_pre = g_switch_head_status;
		g_switch_back_status_pre = g_switch_back_status;
	}
	
	
	if((g_head_stop)&&(dir_temp == 0))
	is_ok_to_run=0;
	else if ((g_back_stop)&&(dir_temp == 1))
	is_ok_to_run=0;
	
	
	if(is_ok_to_run == 1)
	HAL_GPIO_WritePin(p->EN_GPIOx, p->EN_Pin, GPIO_PIN_RESET);
	else 
	HAL_GPIO_WritePin(p->EN_GPIOx, p->EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief ֹͣ�������
 * 
 * @param CS Ƭѡ
 */
void Stop_STEP_Motor(SPI_CS_TypeDef *CS)		
{
	HAL_GPIO_WritePin(CS->EN_GPIOx, CS->EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief ���ֵ����״̬w
 * 
 * @param CS Ƭѡ
 */
void check_motor_status(SPI_CS_TypeDef *CS)
{
 	SPI_CS_TypeDef *p = CS;
	
	if ((p->dir == 0) && (HAL_GPIO_ReadPin(p->pos_limit->GPIOx_head, p->pos_limit->Pin_head) == 0))	
	Stop_STEP_Motor(p);
	else if ((p->dir == 1) && (HAL_GPIO_ReadPin(p->pos_limit->GPIOx_back, p->pos_limit->Pin_back) == 0))
	Stop_STEP_Motor(p);
}

/**
 * @brief �ı����ٶ�
 * 
 * @param CS Ƭѡ
 * @param new_speed �趨�ٶ�
 */
void Change_Motor_Speed(SPI_CS_TypeDef *CS, uint32_t new_speed)
{
	SPI_CS_TypeDef *p = CS;
	uint32_t new_speed_temp = new_speed;
	CS->vmax = new_speed_temp;
	sendData_SPI1(REG_VMAX, p->vmax, p);     				//PAGE35:VMAX�ٶ�ģʽ�µ�Ŀ���ٶ�                               2^23-512 
}

/**
 * @brief ��ȡ�ٶ�
 * 
 * @param CS Ƭѡ
 * @return uint32_t �ٶ�
 */
uint32_t Get_Speed(SPI_CS_TypeDef *CS)
{
	uint32_t speed;
	ReadData_SPI1(0X22, CS);
	speed = ReadData_SPI1(0X22, CS);
	return speed;
}





