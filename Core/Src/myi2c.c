/**
 * @file myi2c.c
 * @author Zhao Haofei
 * @brief 软件i2c
 * @version 1.0
 * @date 2023-05-10
 *
 * @copyright Copyright (c) 2023
 */
#include "myi2c.h"
#include "delay.h"

/*********************************************** AS5048B *******************************************************/
void i2c_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    I2C_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    I2C_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = I2C_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* 快速 */
    HAL_GPIO_Init(I2C_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = I2C_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    i2c_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void i2c_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us的延时 */   
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void i2c_start(void)
{
    I2C_SDA(1);
    I2C_SCL(1);
    i2c_delay();
	
    I2C_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    i2c_delay();
	
    I2C_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    i2c_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void i2c_stop(void)
{
    I2C_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    i2c_delay();
    I2C_SCL(1);
    i2c_delay();
	
    I2C_SDA(1);     /* 发送I2C总线结束信号 */
    i2c_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t i2c_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    I2C_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    i2c_delay();
    I2C_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    i2c_delay();

    while (I2C_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 500)
        {
            i2c_stop();
            rack = 1;
            break;
        }
    }

    I2C_SCL(0);     /* SCL=0, 结束ACK检查 */
    i2c_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void i2c_ack(void)
{
    I2C_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    i2c_delay();

    I2C_SCL(1);     /* 产生一个时钟 */
    i2c_delay();
    I2C_SCL(0);
    i2c_delay();

    I2C_SDA(1);     /* 主机释放SDA线 */
    i2c_delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void i2c_nack(void)
{
    I2C_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    i2c_delay();

    I2C_SCL(1);     /* 产生一个时钟 */
    i2c_delay();
    I2C_SCL(0);
    i2c_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void i2c_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        I2C_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        i2c_delay();
        I2C_SCL(1);
        i2c_delay();
        I2C_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    I2C_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        I2C_SCL(1);
        i2c_delay();

        if (I2C_READ_SDA)
        {
            receive++;
        }
        
        I2C_SCL(0);
        i2c_delay();
    }

    if (!ack)
    {
        i2c_nack();     /* 发送nACK */
    }
    else
    {
        i2c_ack();      /* 发送ACK */
    }

    return receive;
}
/*********************************************** PCA9539 *******************************************************/
//I2C_HandleTypeDef hi2c3;
GPIO_InitTypeDef gpio_init_struct;
/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
void iic_init(void)
{
//    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 快速 */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us的延时 */   
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
	
    IIC_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
	
    IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
	
    IIC_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
    IIC_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 500)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();

    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();

    IIC_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();

    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(1);
        iic_delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }
        
        IIC_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic_nack();     /* 发送nACK */
    }
    else
    {
        iic_ack();      /* 发送ACK */
    }

    return receive;
}

/********************************************* FM24C16 ***********************************************/

void iic2_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC2_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC2_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC2_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                 /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 快速 */
    HAL_GPIO_Init(IIC2_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC2_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;         /* 开漏输出 */
    HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic2_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic2_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us的延时 */   
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic2_start(void)
{
    IIC2_SDA(1);
    IIC2_SCL(1);
    iic2_delay();
	
    IIC2_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic2_delay();
	
    IIC2_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic2_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic2_stop(void)
{
    IIC2_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic2_delay();
    IIC2_SCL(1);
    iic2_delay();
	
    IIC2_SDA(1);     /* 发送I2C总线结束信号 */
    iic2_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic2_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    IIC2_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic2_delay();
    IIC2_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic2_delay();

    while (IIC2_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 500)
        {
            iic2_stop();
            rack = 1;
            break;
        }
    }

    IIC2_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic2_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic2_ack(void)
{
    IIC2_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic2_delay();

    IIC2_SCL(1);     /* 产生一个时钟 */
    iic2_delay();
    IIC2_SCL(0);
    iic2_delay();

    IIC2_SDA(1);     /* 主机释放SDA线 */
    iic2_delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic2_nack(void)
{
    IIC2_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic2_delay();

    IIC2_SCL(1);     /* 产生一个时钟 */
    iic2_delay();
    IIC2_SCL(0);
    iic2_delay();
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic2_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC2_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic2_delay();
        IIC2_SCL(1);
        iic2_delay();
        IIC2_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC2_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic2_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC2_SCL(1);
        iic2_delay();

        if (IIC2_READ_SDA)
        {
            receive++;
        }
        
        IIC2_SCL(0);
        iic2_delay();
    }

    if (!ack)
    {
        iic2_nack();     /* 发送nACK */
    }
    else
    {
        iic2_ack();      /* 发送ACK */
    }

    return receive;
}







