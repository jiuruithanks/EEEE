/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Tmc5160_H
#define __Tmc5160_H
#include "main.h"
#include "spi.h"
#include "bsp_as5048b.h"

/* д���ʵ�ַ�������0x80 */
#define REG_GCONF           0x80 
#define REG_IHOLD_IRUN      0x90    ///4+5+5  iholddelay irun(0~31) ihold (0~31)  
#define REG_TPOWERDOWN      0x91 
#define REG_TSTEP           0X92
#define REG_TPWMTHRS        0X93
#define REG_TCOOLTHRS       0X94
#define REG_THIGH           0X95
#define REG_MODE            0xA0
#define REG_XACTUAL         0XA1 ///ʵ��λ��
#define REG_VACTUAL         0XA2 ///ʵ���ٶ�
#define REG_VSTART          0XA3 ///�����ٶ�
#define REG_A1              0XA4 ///
#define REG_V1              0XA5 ///
#define REG_AMAX            0XA6 ///
#define REG_VMAX            0XA7 ///    ʵ�ʵ�λ����page:76
#define REG_DMAX            0XA8 ///
#define REG_D1              0XAA ///
#define REG_VSTOP           0XAB ///
#define REG_CHOPCONF        0XEC //ն���Ĵ���
#define REG_PWMCONF         0XF0 //��ѹ�������ģʽstealthChop




typedef struct {
GPIO_TypeDef  *GPIOx_head;    ///SPI Ƭѡ�ź�
uint32_t Pin_head; 
GPIO_TypeDef  *GPIOx_back;    ///SPI Ƭѡ�ź�
uint32_t Pin_back; 
}MOTOR_LIMIT_TypeDef;


extern MOTOR_LIMIT_TypeDef STEP_MOTOR4;
extern MOTOR_LIMIT_TypeDef STEP_MOTOR5;

typedef struct {
GPIO_TypeDef  *GPIOx;       //SPI Ƭѡ�ź�
uint32_t Pin; 

GPIO_TypeDef  *EN_GPIOx;    //TMC5160 ʹ�� Ƭѡ�ź�
uint32_t EN_Pin;            

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

AS5048B_HandleTypeDef *has5048b;//�Ŵ�������ѡ��
MOTOR_LIMIT_TypeDef *pos_limit;//λ������
uint32_t vmax_default; 		//�˶�б��Ŀ���ٶȵ�Ĭ��ֵ

}SPI_CS_TypeDef;



extern SPI_CS_TypeDef SPICS1;
extern SPI_CS_TypeDef SPICS2;
extern SPI_CS_TypeDef SPICS3;
extern SPI_CS_TypeDef SPICS4;
extern SPI_CS_TypeDef SPICS5;
extern MOTOR_LIMIT_TypeDef STEP_MOTOR4_LIMIT;
extern MOTOR_LIMIT_TypeDef STEP_MOTOR5_LIMIT;
extern uint8_t Step_Motor_Flag[8];
extern uint8_t Step_Motor_Flag_pre[8];
extern uint8_t g_delay_flag;


/***************************************** ��ʼ����Ӳ������ ********************************************/

/***************************************** ��̬���� ********************************************/
static char SPI_ReceiveByte(char Send_Data);

/***************************************** ��ͨ���� ********************************************/
void Spi_Cs_init(void);									
void Tmc5160Initial(SPI_CS_TypeDef *CS); //TMC5160��ʼ��

void sendData_SPI1(unsigned long address, long datagram, SPI_CS_TypeDef *CS);
long ReadData_SPI1(char address,SPI_CS_TypeDef *CS);
void Run_All_Step_Motor(void);	
void Dis_All_Step_Driver(void);
void EN_All_Step_Driver(void);

void Run_STEP_Motor_POS(SPI_CS_TypeDef *CS, uint8_t tar_pos);
void Run_STEP_Motor_SPEED(SPI_CS_TypeDef *CS , int dir);	
void Stop_STEP_Motor(SPI_CS_TypeDef *CS);
void check_motor_status(SPI_CS_TypeDef *CS);
void Change_Motor_Speed(SPI_CS_TypeDef *CS, uint32_t new_speed);
uint32_t Get_Speed(SPI_CS_TypeDef *CS);



#endif

