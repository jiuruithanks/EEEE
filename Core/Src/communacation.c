#include "communacation.h"
#include <string.h>
#include "modbus_crc16.h"

uint8_t ucSendDataBuff[10]={0};

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/*-------串口参数-----------------------------------------------*/

#define BUFFER_SIZE     32                    //由协议求得
uint8_t ucDMAReceiveBuff[BUFFER_SIZE] = {0};  //DMA缓冲区数组

uint8_t ucFramBuff[BUFFER_SIZE];//抽出的数组


/*---------报文参数----------------------------------------------*/
uint8_t  ucFramHead;                //包头
uint8_t  ucFramLength;              //包长度
uint8_t  ucFramRecever;             //接收者
uint8_t  ucFramType;                //包类型
uint8_t  ucEscKey;                  //退出按键
uint8_t  ucOptKey;                  //操作按键
uint8_t  ucEnterKey;                //进入按键
uint8_t  ucDsDcOpenOrClose;         //导丝DC开合
uint8_t  ucDsBack;                  //导丝退
uint8_t  ucDsFront;                 //导丝进
uint8_t  usQxDcOpenOrClose;         //器械DC开合
uint8_t  ucQxBack;                  //器械退
uint8_t  ucQxFront;                 //器械进
uint8_t  ucDangWei;                 //档位
uint16_t usCRC;


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


/**
 * @brief 拆分字符进行解码
 * @param  *ucUNTranslateBuff  抽出的数组
 * @param  ucDataLength        上述数组的字节长度
 * @return none
 * @note   
 */	
void vByteSpot( uint8_t *ucBuff )
{
    uint8_t ucFramBuff[19];   //转义后的数组-长度由协议固定

    /*1.CRC校验*/
    usCRC =  ((uint16_t)ucFramBuff[15] <<8) | ucFramBuff[14];        

    if(ucCheckCRC(ucFramBuff,usCRC)==0){return;}

    /*2.逐字节解析*/
    ucFramHead = ucFramBuff[0];    //包头
    ucFramLength = ucFramBuff[1];  //包长度
    ucFramRecever = ucFramBuff[2]; //接收者
    ucFramType = ucFramBuff[3];    //包类型
    
    ucEscKey = ucFramBuff[4];                   //退出按键
    ucOptKey = ucFramBuff[5];                   //操作按键
    ucEnterKey = ucFramBuff[6];                 //进入按键
    ucDsDcOpenOrClose = ucFramBuff[7];          //导丝DC开合
    ucDsBack = ucFramBuff[8];                   //导丝退
    ucDsFront = ucFramBuff[9];                  //导丝进
    usQxDcOpenOrClose = ucFramBuff[10];         //器械DC开合
    ucQxBack = ucFramBuff[11];                  //器械退
    ucQxFront = ucFramBuff[12];                 //器械进
    ucDangWei = ucFramBuff[13];                 //档位
    
    
    


}

/**
 * @brief  检查CRC是否正确
 * @param  *ucFramBuff  经过转义之后的数组
 * @param  *usCRC  			串口传递过来的CRC
 * @return 1-正确 0-有误
 * @note   
 */	
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC)
{
	 if(modbus_crc16(ucFramBuff, 17) == usCRC) //长度由协议制定
	 {
		 return 1;
	 }
	 else return 0;
	
} 

