/**
 * @file myi2c.c
 * @author Zhao Haofei
 * @brief ���i2c
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

    I2C_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    I2C_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = I2C_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* ���� */
    HAL_GPIO_Init(I2C_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = I2C_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* ��©��� */
    HAL_GPIO_Init(I2C_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    i2c_stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void i2c_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us����ʱ */   
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void i2c_start(void)
{
    I2C_SDA(1);
    I2C_SCL(1);
    i2c_delay();
	
    I2C_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    i2c_delay();
	
    I2C_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    i2c_delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void i2c_stop(void)
{
    I2C_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    i2c_delay();
    I2C_SCL(1);
    i2c_delay();
	
    I2C_SDA(1);     /* ����I2C���߽����ź� */
    i2c_delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t i2c_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    I2C_SDA(1);     /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    i2c_delay();
    I2C_SCL(1);     /* SCL=1, ��ʱ�ӻ����Է���ACK */
    i2c_delay();

    while (I2C_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 500)
        {
            i2c_stop();
            rack = 1;
            break;
        }
    }

    I2C_SCL(0);     /* SCL=0, ����ACK��� */
    i2c_delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void i2c_ack(void)
{
    I2C_SDA(0);     /* SCL 0 -> 1 ʱ SDA = 0,��ʾӦ�� */
    i2c_delay();

    I2C_SCL(1);     /* ����һ��ʱ�� */
    i2c_delay();
    I2C_SCL(0);
    i2c_delay();

    I2C_SDA(1);     /* �����ͷ�SDA�� */
    i2c_delay();
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void i2c_nack(void)
{
    I2C_SDA(1);     /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    i2c_delay();

    I2C_SCL(1);     /* ����һ��ʱ�� */
    i2c_delay();
    I2C_SCL(0);
    i2c_delay();
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void i2c_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        I2C_SDA((data & 0x80) >> 7);    /* ��λ�ȷ��� */
        i2c_delay();
        I2C_SCL(1);
        i2c_delay();
        I2C_SCL(0);
        data <<= 1;     /* ����1λ,������һ�η��� */
    }
    I2C_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        i2c_nack();     /* ����nACK */
    }
    else
    {
        i2c_ack();      /* ����ACK */
    }

    return receive;
}
/*********************************************** PCA9539 *******************************************************/
//I2C_HandleTypeDef hi2c3;
GPIO_InitTypeDef gpio_init_struct;
/**
 * @brief       ��ʼ��IIC
 * @param       ��
 * @retval      ��
 */
void iic_init(void)
{
//    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ���� */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* ��©��� */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    iic_stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void iic_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us����ʱ */   
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
	
    IIC_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    iic_delay();
	
    IIC_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    iic_delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
	
    IIC_SDA(1);     /* ����I2C���߽����ź� */
    iic_delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t iic_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);     /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    iic_delay();
    IIC_SCL(1);     /* SCL=1, ��ʱ�ӻ����Է���ACK */
    iic_delay();

    while (IIC_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 500)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, ����ACK��� */
    iic_delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1 ʱ SDA = 0,��ʾӦ�� */
    iic_delay();

    IIC_SCL(1);     /* ����һ��ʱ�� */
    iic_delay();
    IIC_SCL(0);
    iic_delay();

    IIC_SDA(1);     /* �����ͷ�SDA�� */
    iic_delay();
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    iic_delay();

    IIC_SCL(1);     /* ����һ��ʱ�� */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* ��λ�ȷ��� */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* ����1λ,������һ�η��� */
    }
    IIC_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        iic_nack();     /* ����nACK */
    }
    else
    {
        iic_ack();      /* ����ACK */
    }

    return receive;
}

/********************************************* FM24C16 ***********************************************/

void iic2_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC2_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    IIC2_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = IIC2_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;         /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                 /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* ���� */
    HAL_GPIO_Init(IIC2_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC2_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;         /* ��©��� */
    HAL_GPIO_Init(IIC2_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    iic2_stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void iic2_delay(void)
{
//    HAL_Delay(2);
	tim_delay_us(2);     /* 2us����ʱ */   
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void iic2_start(void)
{
    IIC2_SDA(1);
    IIC2_SCL(1);
    iic2_delay();
	
    IIC2_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    iic2_delay();
	
    IIC2_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    iic2_delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void iic2_stop(void)
{
    IIC2_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    iic2_delay();
    IIC2_SCL(1);
    iic2_delay();
	
    IIC2_SDA(1);     /* ����I2C���߽����ź� */
    iic2_delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t iic2_wait_ack(void)
{
    uint16_t waittime = 0;
    uint8_t rack = 0;

    IIC2_SDA(1);     /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    iic2_delay();
    IIC2_SCL(1);     /* SCL=1, ��ʱ�ӻ����Է���ACK */
    iic2_delay();

    while (IIC2_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 500)
        {
            iic2_stop();
            rack = 1;
            break;
        }
    }

    IIC2_SCL(0);     /* SCL=0, ����ACK��� */
    iic2_delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic2_ack(void)
{
    IIC2_SDA(0);     /* SCL 0 -> 1 ʱ SDA = 0,��ʾӦ�� */
    iic2_delay();

    IIC2_SCL(1);     /* ����һ��ʱ�� */
    iic2_delay();
    IIC2_SCL(0);
    iic2_delay();

    IIC2_SDA(1);     /* �����ͷ�SDA�� */
    iic2_delay();
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic2_nack(void)
{
    IIC2_SDA(1);     /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    iic2_delay();

    IIC2_SCL(1);     /* ����һ��ʱ�� */
    iic2_delay();
    IIC2_SCL(0);
    iic2_delay();
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void iic2_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC2_SDA((data & 0x80) >> 7);    /* ��λ�ȷ��� */
        iic2_delay();
        IIC2_SCL(1);
        iic2_delay();
        IIC2_SCL(0);
        data <<= 1;     /* ����1λ,������һ�η��� */
    }
    IIC2_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t iic2_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++)    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        iic2_nack();     /* ����nACK */
    }
    else
    {
        iic2_ack();      /* ����ACK */
    }

    return receive;
}







