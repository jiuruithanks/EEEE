#include "pca9539.h"

/* 寄存器地址 */
#define RegAddr_Input_Port0                    0x00  
#define RegAddr_Input_Port1                    0x01  
#define RegAddr_Output_Port0                   0x02  
#define RegAddr_Output_Port1                   0x03  
#define RegAddr_PolarityInvert_Port0           0x04  
#define RegAddr_PolarityInvert_Port1           0x05  
#define RegAddr_Config_Port0                   0x06  
#define RegAddr_Config_Port1                   0x07  

/* 寄存器初始值 */
#define PCA9539_Reg_Input_Port0_Default_Value                0xFF
#define PCA9539_Reg_Input_Port0_Default_Value                0xFF
#define PCA9539_Reg_Output_Port0_Default_Value               0x00
#define PCA9539_Reg_Output_Port1_Default_Value               0x00
#define PCA9539_Reg_PolarityInvert_Port0_Default_Value       0x00
#define PCA9539_Reg_PolarityInvert_Port1_Default_Value       0x00
#define PCA9539_Reg_Config_Port0_Default_Value               0xFF
#define PCA9539_Reg_Config_Port1_Default_Value               0xFF

PCA9539_Reg_Typedef PCA9539_RegStruct = 
{
	PCA9539_Reg_Input_Port0_Default_Value                  ,
	PCA9539_Reg_Input_Port0_Default_Value                  ,
	PCA9539_Reg_Output_Port0_Default_Value                 ,
	PCA9539_Reg_Output_Port1_Default_Value                 ,
	PCA9539_Reg_PolarityInvert_Port0_Default_Value         ,
	PCA9539_Reg_PolarityInvert_Port1_Default_Value         ,
	PCA9539_Reg_Config_Port0_Default_Value                 ,
	PCA9539_Reg_Config_Port1_Default_Value                 ,
};

/* USER CODE BEGIN */
BSP_I2Cx_Typedef BSP_I2Cx_PCA9539=
{
	I2C3,
	0x74,
};

PCA9539_Init_Typedef PCA9539_InitStruct=
{
	&BSP_I2Cx_PCA9539,
	&PCA9539_RegStruct,
	PCA9539_INT_GPIO_Port,
	PCA9539_INT_Pin,
};

/**
  * @brief  PCA9539读取触发引脚电平
  * @param  PCA9539_InitStruct 外设结构体
  * @retval 返回值:
  *          - PCA9539_Port_x_Reset     : 引脚低电平
  *          - PCA9539_Port_x_Set       : 引脚高电平
  */
uint8_t PCA9539_IsIntPinSet(PCA9539_Init_Typedef *PCA9539_InitStruct)
{
	if(HAL_GPIO_ReadPin(PCA9539_InitStruct->Int_GPIOx,PCA9539_InitStruct->Int_Pin))
	{
		return PCA9539_Port_x_Set;
	}
	else
	{
		return PCA9539_Port_x_Reset;
	}
}
/* USER CODE END */

/**
  * @brief  PCA9539初始化
  * @param  PCA9539_InitStruct 外设结构体
  * @retval 返回值:
  *          - PCA9539_ERROR     : 初始化失败
  *          - PCA9539_SUCCESS   : 没有发成功
  */
PCA9539_ErrorStatus PCA9539_Init (PCA9539_Init_Typedef *PCA9539_InitStruct)
{
	/* 初始化 */
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if(BSP_I2Cx_Init(PCA9539_InitStruct->PCA9539_I2CStruct) == I2C_ERROR) return PCA9539_ERROR;
	
	/* 输出寄存器默认值 */
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_Output_Port0,1,&pPCA9539_Reg_Handle->Output_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;

	/* 极性寄存器默认值 */
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_PolarityInvert_Port0,1,&pPCA9539_Reg_Handle->PolarityInvert_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;

	/* 配置寄存器默认值 */
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_Config_Port0,1,&pPCA9539_Reg_Handle->Config_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;

	return PCA9539_SUCCESS;
}

/**
  * @brief  PCA9539引脚输入输出配置
  * @param  PCA9539_InitStruct       外设结构体
  * @param  PCA9539_Config_Typedef   寄存器结构体
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_Config_Pin (PCA9539_Init_Typedef *PCA9539_InitStruct, PCA9539_Config_Typedef *PCA9539_Config_Struct )
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if(PCA9539_Config_Struct->Port == PCA9539_PORT_0)
	{
		if( PCA9539_Config_Struct->Mode == PCA9539_MODE_OUTPUT)
		{
			pPCA9539_Reg_Handle->Config_Port0.Reg_Value &= (~PCA9539_Config_Struct->Pin);			
		}
		else
		{
			pPCA9539_Reg_Handle->Config_Port0.Reg_Value |= (PCA9539_Config_Struct->Pin);	
		}	
	}
	else if(PCA9539_Config_Struct->Port == PCA9539_PORT_1)
	{
		if( PCA9539_Config_Struct->Mode == PCA9539_MODE_OUTPUT)
		{
			pPCA9539_Reg_Handle->Config_Port1.Reg_Value &= (~PCA9539_Config_Struct->Pin);			
		}
		else
		{
			pPCA9539_Reg_Handle->Config_Port1.Reg_Value |= (PCA9539_Config_Struct->Pin);	
		}	
	}
	
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_Config_Port0,1,&pPCA9539_Reg_Handle->Config_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;
	
	return PCA9539_SUCCESS;
}

/**
  * @brief  PCA9539输入引脚极性反装
  * @param  PCA9539_InitStruct       外设结构体
  * @param  PCA9539_PORT_x     端口号	
  *          - PCA9539_PORT_0
  *          - PCA9539_PORT_1
  * @param  PCA9539_PIN_x      引脚
  *          - PCA9539_PIN_0	
  *          - PCA9539_PIN_1	
  *          - PCA9539_PIN_2	
  *          - PCA9539_PIN_3	
  *          - PCA9539_PIN_4	
  *          - PCA9539_PIN_5	
  *          - PCA9539_PIN_6	
  *          - PCA9539_PIN_7
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_SetPolarityInputPin (PCA9539_Init_Typedef *PCA9539_InitStruct, uint32_t PCA9539_PORT_x, uint32_t PCA9539_PIN_x )
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if( PCA9539_PORT_x == PCA9539_PORT_0 )
	{
		pPCA9539_Reg_Handle->PolarityInvert_Port0.Reg_Value |= (PCA9539_PIN_x);	
	}
	else if( PCA9539_PORT_x == PCA9539_PORT_1 )
	{
		pPCA9539_Reg_Handle->PolarityInvert_Port1.Reg_Value |= (PCA9539_PIN_x);
	}
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_PolarityInvert_Port0,1,&pPCA9539_Reg_Handle->PolarityInvert_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;	

	return PCA9539_SUCCESS;
}

/**
  * @brief  PCA9539输入引脚极性反装
  * @param  PCA9539_InitStruct       外设结构体
  * @param  PCA9539_PORT_x     端口号	
  *          - PCA9539_PORT_0
  *          - PCA9539_PORT_1
  * @param  PCA9539_PIN_x      引脚
  *          - PCA9539_PIN_0	
  *          - PCA9539_PIN_1	
  *          - PCA9539_PIN_2	
  *          - PCA9539_PIN_3	
  *          - PCA9539_PIN_4	
  *          - PCA9539_PIN_5	
  *          - PCA9539_PIN_6	
  *          - PCA9539_PIN_7
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_ResetPolarityInputPin (PCA9539_Init_Typedef *PCA9539_InitStruct, uint32_t PCA9539_PORT_x, uint32_t PCA9539_PIN_x )
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if( PCA9539_PORT_x == PCA9539_PORT_0 )
	{
		pPCA9539_Reg_Handle->PolarityInvert_Port0.Reg_Value &= (~PCA9539_PIN_x);	
	}
	else if( PCA9539_PORT_x == PCA9539_PORT_1 )
	{
		pPCA9539_Reg_Handle->PolarityInvert_Port1.Reg_Value &= (~PCA9539_PIN_x);
	}
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_PolarityInvert_Port0,1,&pPCA9539_Reg_Handle->PolarityInvert_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;	
	return PCA9539_SUCCESS;
	
}

/**
  * @brief  PCA9539引脚输出置位
  * @param  PCA9539_InitStruct 外设结构体
  * @param  PCA9539_PORT_x     端口号	
  *          - PCA9539_PORT_0
  *          - PCA9539_PORT_1
  * @param  PCA9539_PIN_x      引脚
  *          - PCA9539_PIN_0	
  *          - PCA9539_PIN_1	
  *          - PCA9539_PIN_2	
  *          - PCA9539_PIN_3	
  *          - PCA9539_PIN_4	
  *          - PCA9539_PIN_5	
  *          - PCA9539_PIN_6	
  *          - PCA9539_PIN_7
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_SetOutputPin (PCA9539_Init_Typedef *PCA9539_InitStruct, uint32_t PCA9539_PORT_x, uint32_t PCA9539_PIN_x)
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if( PCA9539_PORT_x == PCA9539_PORT_0 )
	{
		pPCA9539_Reg_Handle->Output_Port0.Reg_Value |= (PCA9539_PIN_x);	
	}
	else if( PCA9539_PORT_x == PCA9539_PORT_1 )
	{
		pPCA9539_Reg_Handle->Output_Port1.Reg_Value |= (PCA9539_PIN_x);
	}
	
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_Output_Port0,1,&pPCA9539_Reg_Handle->Output_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;

	return PCA9539_SUCCESS;
	
}

/**
  * @brief  PCA9539引脚输出置零
  * @param  PCA9539_InitStruct 外设结构体
  * @param  PCA9539_PORT_x     端口号	
  *          - PCA9539_PORT_0
  *          - PCA9539_PORT_1
  * @param  PCA9539_PIN_x      引脚
  *          - PCA9539_PIN_0	
  *          - PCA9539_PIN_1	
  *          - PCA9539_PIN_2	
  *          - PCA9539_PIN_3	
  *          - PCA9539_PIN_4	
  *          - PCA9539_PIN_5	
  *          - PCA9539_PIN_6	
  *          - PCA9539_PIN_7
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_ResetOutputPin (PCA9539_Init_Typedef *PCA9539_InitStruct, uint32_t PCA9539_PORT_x, uint32_t PCA9539_PIN_x )
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if( PCA9539_PORT_x == PCA9539_PORT_0 )
	{
		pPCA9539_Reg_Handle->Output_Port0.Reg_Value &= (~PCA9539_PIN_x);	
	}
	else if( PCA9539_PORT_x == PCA9539_PORT_1 )
	{
		pPCA9539_Reg_Handle->Output_Port1.Reg_Value &= (~PCA9539_PIN_x);
	}
	
	if(BSP_I2C_Write_Bytes(pI2C_Handle,RegAddr_Output_Port0,1,&pPCA9539_Reg_Handle->Output_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;	

	return PCA9539_SUCCESS;
}

/**
  * @brief  PCA9539读取所有端口电平
  * @param  PCA9539_InitStruct 外设结构体
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_ReadInputPorts (PCA9539_Init_Typedef *PCA9539_InitStruct)
{
	BSP_I2Cx_Typedef *pI2C_Handle;
	PCA9539_Reg_Typedef *pPCA9539_Reg_Handle;
	
	pI2C_Handle = PCA9539_InitStruct->PCA9539_I2CStruct;
	pPCA9539_Reg_Handle = PCA9539_InitStruct->PCA9539_RegStruct;	
	
	if(BSP_I2C_Read_Bytes(pI2C_Handle,RegAddr_Input_Port0,1,&pPCA9539_Reg_Handle->Input_Port0.Reg_Value,2) == I2C_ERROR) return PCA9539_ERROR;	

	return PCA9539_SUCCESS;
}

/**
  * @brief  PCA9539外部调用函数
  * @param  PCA9539_InitStruct 外设结构体
  * @retval 返回值:
  *          - PCA9539_ERROR     : 失败
  *          - PCA9539_SUCCESS   : 成功
  */
PCA9539_ErrorStatus PCA9539_IntHandle (PCA9539_Init_Typedef *PCA9539_InitStruct)
{
	if( !PCA9539_IsIntPinSet(PCA9539_InitStruct) )/* 低电平触发 */
	{
		if(PCA9539_ReadInputPorts(PCA9539_InitStruct) == PCA9539_ERROR) return PCA9539_ERROR;
	}
	return PCA9539_SUCCESS;
}
















