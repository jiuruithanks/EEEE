#include "communacation.h"
#include <string.h>
#include "modbus_crc16.h"

uint8_t ucSendDataBuff[10]={0};

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern PCA9555_HandleTypeDef PCA9555_KEY;
/*-------串口参数-----------------------------------------------*/

#define BUFFER_SIZE     32                    //由协议求得
uint8_t ucDMAReceiveBuff[BUFFER_SIZE] = {0};  //DMA缓冲区数组

uint8_t ucFramBuff[BUFFER_SIZE/2];//抽出的数组

/*-------报文-------------------------------------------------*/
xFramDecode_TypeDef FramDecode;
xFramDecode_TypeDef LastFramDecode;

/*----------函数----------------------------------------------------------*/
/**
 * @brief  开启接收DMA
 * @param  none
 * @return none
 * @note   none
 */	
void vStartReceive(void)
{
 UART_Start_Receive_DMA(&huart1, ucDMAReceiveBuff, BUFFER_SIZE);
}


/**
 * @brief  DMA缓冲区满中断回调
 * @param  ucUNTranslateBuff  从串口收到的原始字符
 * @return none
 * @note   
 */	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ///	HAL_UART_DMAStop(&huart2);

    uint8_t index1 = 0, index2 = 0, flag = 0;

    // 查找0x02包头
    for(int i = 0; i < BUFFER_SIZE; i++) 
    {
        if(ucDMAReceiveBuff[i] == 0x55) 
        {
            if(flag == 0) 
                {
                    index1 = i;
                    flag = 1;
                } 
                else if(flag == 1) 
                {
                    index2 = i;
                    break;
                }
        }
    }

    // 清空ucEscFramBuff数组
    memset(ucFramBuff, 0, BUFFER_SIZE);

    // 检查index1和index2是否合法
    if(flag == 1 && index2 > index1 && index2 < BUFFER_SIZE) 
    {
        // 拷贝数据
        for(int i = 0; index1 + i < index2; i++) 
        {	
            ucFramBuff[i] = ucDMAReceiveBuff[index1 + i];
        }
    } 
    else 
    {
    // 处理错误情况
    }


    /*启动响应*/
    vByteSpot(ucFramBuff);
        

        
}

void vLightLED(xFramDecode_TypeDef xFramDecode)
{
    uint16_t usTemp = 0;
    uint8_t ucFramToSend[10]={0xAA,0x08,0x02,0x01,  0,0,0,0,  0,0};
    
    /*1.组装*/
    if((xFramDecode.ucOptKey == 1 ))//&&(LastFramDecode.ucOptKey == 0))
    {
     ucFramToSend[4] = 1;
    }
    else ucFramToSend[4] = 0; 
    
    if((xFramDecode.ucDsDcOpenOrClose == 1 ))//&&(LastFramDecode.ucDsDcOpenOrClose == 0))
    {
     ucFramToSend[5] = 1;
    }
    else ucFramToSend[5] = 0; 
    
    if((xFramDecode.usQxDcOpenOrClose == 1 ))//&&(LastFramDecode.usQxDcOpenOrClose == 0))
    {
     ucFramToSend[6] = 1;
    }
    else ucFramToSend[6] = 0; 
     
    if((xFramDecode.ucDangWei >= 1))//&&(LastFramDecode.ucDangWei <= 5))
    {
     ucFramToSend[7] = xFramDecode.ucDangWei;
    }
    else ucFramToSend[7] = 0; 
    
    /*2.计算CRC*/
    usTemp = modbus_crc16(ucFramToSend, 8);
    ucFramToSend[8] =  (uint8_t)(usTemp &0x00FF);
    ucFramToSend[9] =  (uint8_t)((usTemp &0xFF00)>>8);
    
    /*3.发送回去*/
    HAL_UART_Transmit(&huart1,ucFramToSend,10,100);
}


/**
 * @brief 拆分字符进行解码
 * @param  *ucUNTranslateBuff  抽出的数组
 * @param  ucDataLength        上述数组的字节长度
 * @return none
 * @note   
 */	
void vByteSpot( uint8_t *ucFramBuff )
{

    /*1.CRC校验*/
    FramDecode.usCRC =  ((uint16_t)ucFramBuff[15] <<8) | ucFramBuff[14];        

    if(ucCheckCRC(ucFramBuff,FramDecode.usCRC)==0){return;}

    /*2.逐字节解析*/
    FramDecode.ucFramHead = ucFramBuff[0];    //包头
    FramDecode.ucFramLength = ucFramBuff[1];  //包长度
    FramDecode.ucFramRecever = ucFramBuff[2]; //接收者
    FramDecode.ucFramType = ucFramBuff[3];    //包类型
    
    FramDecode.ucEscKey = ucFramBuff[4];                   //退出按键
    FramDecode.ucOptKey = ucFramBuff[5];                   //操作按键
    FramDecode.ucEnterKey = ucFramBuff[6];                 //进入按键
    FramDecode.ucDsDcOpenOrClose = ucFramBuff[7];          //导丝DC开合
    FramDecode.ucDsBack = ucFramBuff[8];                   //导丝退
    FramDecode.ucDsFront = ucFramBuff[9];                  //导丝进
    FramDecode.usQxDcOpenOrClose = ucFramBuff[10];         //器械DC开合
    FramDecode.ucQxBack = ucFramBuff[11];                  //器械退
    FramDecode.ucQxFront = ucFramBuff[12];                 //器械进
    FramDecode.ucDangWei = ucFramBuff[13];                 //档位
    
    
    
    
    vPca9555_Decode_Uart(&PCA9555_KEY,FramDecode); 
}

/**
 * @brief  检查CRC是否正确
 * @param  *ucFramBuff  经过转义之后的数组
 * @param  *usCRC  			串口传递过来的CRC
 * @return 1-正确 0-有误
 * @note   
 */	    uint16_t usTemp;
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC)
{
   
    usTemp = modbus_crc16(ucFramBuff, 16-2);
	 if(usTemp == usCRC) //长度由协议制定
	 {
		 return 1;
	 }
	 else return 0;
	
} 

