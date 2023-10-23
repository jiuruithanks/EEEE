/**
 * --------------------------------------------------------------------------
 * pca9555.h
 *
 *  Created on: Aug 16, 2020
 *      Author: Jack Lestrohan (c) Cobalt Audio - 2020
 * --------------------------------------------------------------------------
 */

#ifndef INC_PCA9555_H_
#define INC_PCA9555_H_

#include "myi2c.h"



//Device Addresses (A0-A1)
#define PCA9555_ADD_74			0x74
#define PCA9555_ADD_75			0x75
#define PCA9555_ADD_76			0x76
#define PCA9555_ADD_77			0x77


/* Command Bytes */
#define PCA9555_CB_INPUTS_PORTS				0x00
#define PCA9555_CB_INPUTS_PORTS2			0x01
#define PCA9555_CB_OUTPUTS_PORTS			0x02
#define PCA9555_CB_OUTPUTS_PORTS2			0x03
#define PCA9555_CB_POL_INVERT_PORTS			0x04
#define PCA9555_CB_POL_INVERT_PORTS2		0x05
#define PCA9555_CB_CONFIG_PORTS				0x06
#define PCA9555_CB_CONFIG_PORTS2			0x07

/* PCA9539 I2C control register power-up default definitions */
#define PCA9539_OUTPUT_PORT_0_DEFAULT           0xFF    /* PCA9539 Output Port 0 register default             */
#define PCA9539_OUTPUT_PORT_1_DEFAULT           0xFF    /* PCA9539 Output Port 1 register default             */
#define PCA9539_POLARITY_INV_PORT_0_DEFAULT     0x00    /* PCA9539 Polarity Inversion Port 0 register default */
#define PCA9539_POLARITY_INV_PORT_1_DEFAULT     0x00    /* PCA9539 Polarity Inversion Port 1 register default */
#define PCA9539_CONFIG_PORT_0_DEFAULT           0xFF    /* PCA9539 Configuration Port 0 register default      */
#define PCA9539_CONFIG_PORT_1_DEFAULT           0xFF    /* PCA9539 Configuration Port 1 register default      */

/* PCA9539 Port 0 and 1 input or output pin index definition */
#define PCA9539_PIN0_INDEX    0x00    /* PCA9539 Port 0 and 1 Pin Register 0 (P00 or P10) */
#define PCA9539_PIN1_INDEX    0x01    /* PCA9539 Port 0 and 1 Pin Register 1 (P01 or P11) */
#define PCA9539_PIN2_INDEX    0x02    /* PCA9539 Port 0 and 1 Pin Register 2 (P02 or P12) */
#define PCA9539_PIN3_INDEX    0x03    /* PCA9539 Port 0 and 1 Pin Register 3 (P03 or P13) */
#define PCA9539_PIN4_INDEX    0x04    /* PCA9539 Port 0 and 1 Pin Register 4 (P04 or P14) */
#define PCA9539_PIN5_INDEX    0x05    /* PCA9539 Port 0 and 1 Pin Register 5 (P05 or P15) */
#define PCA9539_PIN6_INDEX    0x06    /* PCA9539 Port 0 and 1 Pin Register 6 (P06 or P16) */
#define PCA9539_PIN7_INDEX    0x07    /* PCA9539 Port 0 and 1 Pin Register 7 (P07 or P17) */

/* PCA9539 I/O direction */
#define PCA9539_PIN_MODE_OUTPUT    0x00    /* Pin is configured as ouput                */
#define PCA9539_PIN_MODE_INPUT     0x01    /* Pin is configured as high-impedance input */

/* PCA9539 arbitrary port index */
#define PCA9539_PORT0    0x00    /* Port 0 (P00 - P07) */
#define PCA9539_PORT1    0x01    /* Port 1 (P10 - P17) */

typedef enum {
	PCA9555_POLARITY_NORMAL,
	PCA9555_POLARITY_INVERTED
} PCA9555_PinPolarity_t;

typedef enum {
	PCA9555_PIN_OUTPUT_MODE,
	PCA9555_PIN_INPUT_MODE
} PCA9555_PinMode_t;

typedef enum {
	PCA9555_BIT_RESET,
	PCA9555_BIT_SET
} PCA9555_BitValue_t;

/* pca9555 struct type for multi-instances handling */
typedef struct {
//	I2C_HandleTypeDef*	hi2c;
	GPIO_InitTypeDef*	hi2c;
	uint16_t			addr;				//地址

	uint16_t      		key_out;			///临时存放直接读取的值
	uint16_t      		key_out_pre;		///临时存放上一次直接读取的值
	uint8_t 			key[16];			///解码后对应按键值
	uint8_t 			key_pre[16];
	uint8_t				key_action[16];		//按键动作的标志位
	uint8_t 			rotate;
	uint16_t			rotate_bit;
	uint8_t 			key_equal_flag;
} PCA9555_HandleTypeDef;


extern uint8_t key_update_flag;
extern PCA9555_HandleTypeDef PCA9555_LED;
extern PCA9555_HandleTypeDef PCA9555_KEY;




/******************************************************** 硬件iic **********************************************************/
//HAL_StatusTypeDef pca9555_init(PCA9555_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c, uint16_t addr);
//HAL_StatusTypeDef pca9555_DigitalWrite(PCA9555_HandleTypeDef *hdev, uint8_t pin, GPIO_PinState pinState);
//HAL_StatusTypeDef pca9555_digitalRead(PCA9555_HandleTypeDef *hdev, uint8_t pin, uint8_t *data);
//HAL_StatusTypeDef pca9555_pinMode(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_PinMode_t mode, PCA9555_PinPolarity_t polarity);

/******************************************************** 软件iic **********************************************************/
static void PCA9539_Write_Register(PCA9555_HandleTypeDef *hdev, uint8_t cmd, uint16_t data);
static uint16_t PCA9539_Read_Reg(PCA9555_HandleTypeDef *hdev, uint8_t cmdByte, uint16_t *data);
static HAL_StatusTypeDef PCA9539_updateRegisterBit(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_BitValue_t bitvalue, uint8_t commandByteAddr);

HAL_StatusTypeDef PCA9539_pinMode(PCA9555_HandleTypeDef *hdev, uint8_t pin, PCA9555_PinMode_t mode, PCA9555_PinPolarity_t polarity);
void PCA9539_init(void);
HAL_StatusTypeDef PCA9539_DigitalWrite(PCA9555_HandleTypeDef *hdev, uint8_t pin, GPIO_PinState pinState);
HAL_StatusTypeDef PCA9539_DigitalRead(PCA9555_HandleTypeDef *hdev, uint8_t pin, uint8_t *pinValue);
void led_on(void);
#endif /* INC_PCA9555_H_ */
