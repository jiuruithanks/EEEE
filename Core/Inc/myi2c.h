#ifndef __MYI2C_H
#define __MYI2C_H

#include "main.h"


/*********************************** AS5048B *******************************************************/
/* 引脚 定义 */
#define I2C_SCL_GPIO_PORT               GPIOC
#define I2C_SCL_GPIO_PIN                GPIO_PIN_11
#define I2C_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

#define I2C_SDA_GPIO_PORT               GPIOC
#define I2C_SDA_GPIO_PIN                GPIO_PIN_12
#define I2C_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

/* IO操作 */
#define I2C_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define I2C_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define I2C_READ_SDA     HAL_GPIO_ReadPin(I2C_SDA_GPIO_PORT, I2C_SDA_GPIO_PIN) /* 读取SDA */
#define I2C_READ_SCL     HAL_GPIO_ReadPin(I2C_SCL_GPIO_PORT, I2C_SCL_GPIO_PIN) /* 读取SCL */

/*********************************** PCA9539 *******************************************************/
/* 引脚 定义 */
#define IIC_SCL_GPIO_PORT               GPIOA
#define IIC_SCL_GPIO_PIN                GPIO_PIN_8
#define IIC_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define IIC_SDA_GPIO_PORT               GPIOC
#define IIC_SDA_GPIO_PIN                GPIO_PIN_9
#define IIC_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

/* IO操作 */
#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_GPIO_PIN) /* 读取SDA */
#define IIC_READ_SCL     HAL_GPIO_ReadPin(IIC_SCL_GPIO_PORT, IIC_SCL_GPIO_PIN) /* 读取SCL */
						  
/************************************* FM24C16 *****************************************************/	
/* 引脚 定义 */
#define IIC2_SCL_GPIO_PORT               GPIOB
#define IIC2_SCL_GPIO_PIN                GPIO_PIN_10
#define IIC2_SCL_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define IIC2_SDA_GPIO_PORT               GPIOB
#define IIC2_SDA_GPIO_PIN                GPIO_PIN_11
#define IIC2_SDA_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PC口时钟使能 */

/* IO操作 */
#define IIC2_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC2_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC2_READ_SDA     HAL_GPIO_ReadPin(IIC2_SDA_GPIO_PORT, IIC2_SDA_GPIO_PIN) /* 读取SDA */
#define IIC2_READ_SCL     HAL_GPIO_ReadPin(IIC2_SCL_GPIO_PORT, IIC2_SCL_GPIO_PIN) /* 读取SCL */
						  
extern GPIO_InitTypeDef gpio_init_struct;

/************************************* AS5048B ****************************************/
//IIC1
/* I2C所有操作函数 */
void i2c_init(void);            /* 初始化IIC的IO口 */
void i2c_start(void);           /* 发送IIC开始信号 */
void i2c_stop(void);            /* 发送IIC停止信号 */
void i2c_ack(void);             /* IIC发送ACK信号 */
void i2c_nack(void);            /* IIC不发送ACK信号 */
uint8_t i2c_wait_ack(void);     /* IIC等待ACK信号 */
void i2c_send_byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t i2c_read_byte(unsigned char ack);/* IIC读取一个字节 */
                          
/************************************* PCA9539 ****************************************/
//IIC3
/* IIC3所有操作函数 */
void iic_init(void);            /* 初始化IIC的IO口 */
void iic_start(void);           /* 发送IIC开始信号 */
void iic_stop(void);            /* 发送IIC停止信号 */
void iic_ack(void);             /* IIC发送ACK信号 */
void iic_nack(void);            /* IIC不发送ACK信号 */
uint8_t iic_wait_ack(void);     /* IIC等待ACK信号 */
void iic_send_byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t iic_read_byte(unsigned char ack);/* IIC读取一个字节 */
						
/************************************* FM24C16 ****************************************/
//IIC2
void iic2_init(void);            /* 初始化IIC的IO口 */
void iic2_start(void);           /* 发送IIC开始信号 */
void iic2_stop(void);            /* 发送IIC停止信号 */
void iic2_ack(void);             /* IIC发送ACK信号 */
void iic2_nack(void);            /* IIC不发送ACK信号 */
uint8_t iic2_wait_ack(void);     /* IIC等待ACK信号 */
void iic2_send_byte(uint8_t txd);/* IIC发送一个字节 */
uint8_t iic2_read_byte(unsigned char ack);/* IIC读取一个字节 */
#endif

