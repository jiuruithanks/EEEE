/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Tmc5160_H
#define __Tmc5160_H
#include "main.h"
#include "spi.h"
#include "bsp_as5048b.h"

/* 写访问地址必须加上0x80 */
#define REG_GCONF           0x80 
#define REG_IHOLD_IRUN      0x90    ///4+5+5  iholddelay irun(0~31) ihold (0~31)  
#define REG_TPOWERDOWN      0x91 
#define REG_TSTEP           0X92
#define REG_TPWMTHRS        0X93
#define REG_TCOOLTHRS       0X94
#define REG_THIGH           0X95
#define REG_MODE            0xA0
#define REG_XACTUAL         0XA1 ///实际位置
#define REG_VACTUAL         0XA2 ///实际速度
#define REG_VSTART          0XA3 ///启动速度
#define REG_A1              0XA4 ///
#define REG_V1              0XA5 ///
#define REG_AMAX            0XA6 ///
#define REG_VMAX            0XA7 ///    实际单位换算page:76
#define REG_DMAX            0XA8 ///
#define REG_D1              0XAA ///
#define REG_VSTOP           0XAB ///
#define REG_CHOPCONF        0XEC //斩波寄存器
#define REG_PWMCONF         0XF0 //电压脉宽调制模式stealthChop




typedef struct {
GPIO_TypeDef  *GPIOx_head;    ///SPI 片选信号
uint32_t Pin_head; 
GPIO_TypeDef  *GPIOx_back;    ///SPI 片选信号
uint32_t Pin_back; 
}MOTOR_LIMIT_TypeDef;


extern MOTOR_LIMIT_TypeDef STEP_MOTOR4;
extern MOTOR_LIMIT_TypeDef STEP_MOTOR5;

typedef struct {
GPIO_TypeDef  *GPIOx;       //SPI 片选信号
uint32_t Pin; 

GPIO_TypeDef  *EN_GPIOx;    //TMC5160 使能 片选信号
uint32_t EN_Pin;            

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

AS5048B_HandleTypeDef *has5048b;//磁传感器的选择
MOTOR_LIMIT_TypeDef *pos_limit;//位置限制
uint32_t vmax_default; 		//运动斜坡目标速度的默认值

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


/***************************************** 初始化及硬件配置 ********************************************/

/***************************************** 静态函数 ********************************************/
static char SPI_ReceiveByte(char Send_Data);

/***************************************** 普通函数 ********************************************/
void Spi_Cs_init(void);									
void Tmc5160Initial(SPI_CS_TypeDef *CS); //TMC5160初始化

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

