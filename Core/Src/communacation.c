#include "communacation.h"
#include <string.h>
#include "modbus_crc16.h"

uint8_t ucSendDataBuff[10]={0};

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern PCA9555_HandleTypeDef PCA9555_KEY;
/*-------���ڲ���-----------------------------------------------*/

#define BUFFER_SIZE     32                    //��Э�����
uint8_t ucDMAReceiveBuff[BUFFER_SIZE] = {0};  //DMA����������

uint8_t ucFramBuff[BUFFER_SIZE/2];//���������

/*-------����-------------------------------------------------*/
xFramDecode_TypeDef FramDecode;
xFramDecode_TypeDef LastFramDecode;

/*----------����----------------------------------------------------------*/
/**
 * @brief  ��������DMA
 * @param  none
 * @return none
 * @note   none
 */	
void vStartReceive(void)
{
 UART_Start_Receive_DMA(&huart1, ucDMAReceiveBuff, BUFFER_SIZE);
}


/**
 * @brief  DMA���������жϻص�
 * @param  ucUNTranslateBuff  �Ӵ����յ���ԭʼ�ַ�
 * @return none
 * @note   
 */	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ///	HAL_UART_DMAStop(&huart2);

    uint8_t index1 = 0, index2 = 0, flag = 0;

    // ����0x02��ͷ
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

    // ���ucEscFramBuff����
    memset(ucFramBuff, 0, BUFFER_SIZE);

    // ���index1��index2�Ƿ�Ϸ�
    if(flag == 1 && index2 > index1 && index2 < BUFFER_SIZE) 
    {
        // ��������
        for(int i = 0; index1 + i < index2; i++) 
        {	
            ucFramBuff[i] = ucDMAReceiveBuff[index1 + i];
        }
    } 
    else 
    {
    // ����������
    }


    /*������Ӧ*/
    vByteSpot(ucFramBuff);
        

        
}

void vLightLED(xFramDecode_TypeDef xFramDecode)
{
    uint16_t usTemp = 0;
    uint8_t ucFramToSend[10]={0xAA,0x08,0x02,0x01,  0,0,0,0,  0,0};
    
    /*1.��װ*/
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
    
    /*2.����CRC*/
    usTemp = modbus_crc16(ucFramToSend, 8);
    ucFramToSend[8] =  (uint8_t)(usTemp &0x00FF);
    ucFramToSend[9] =  (uint8_t)((usTemp &0xFF00)>>8);
    
    /*3.���ͻ�ȥ*/
    HAL_UART_Transmit(&huart1,ucFramToSend,10,100);
}


/**
 * @brief ����ַ����н���
 * @param  *ucUNTranslateBuff  ���������
 * @param  ucDataLength        ����������ֽڳ���
 * @return none
 * @note   
 */	
void vByteSpot( uint8_t *ucFramBuff )
{

    /*1.CRCУ��*/
    FramDecode.usCRC =  ((uint16_t)ucFramBuff[15] <<8) | ucFramBuff[14];        

    if(ucCheckCRC(ucFramBuff,FramDecode.usCRC)==0){return;}

    /*2.���ֽڽ���*/
    FramDecode.ucFramHead = ucFramBuff[0];    //��ͷ
    FramDecode.ucFramLength = ucFramBuff[1];  //������
    FramDecode.ucFramRecever = ucFramBuff[2]; //������
    FramDecode.ucFramType = ucFramBuff[3];    //������
    
    FramDecode.ucEscKey = ucFramBuff[4];                   //�˳�����
    FramDecode.ucOptKey = ucFramBuff[5];                   //��������
    FramDecode.ucEnterKey = ucFramBuff[6];                 //���밴��
    FramDecode.ucDsDcOpenOrClose = ucFramBuff[7];          //��˿DC����
    FramDecode.ucDsBack = ucFramBuff[8];                   //��˿��
    FramDecode.ucDsFront = ucFramBuff[9];                  //��˿��
    FramDecode.usQxDcOpenOrClose = ucFramBuff[10];         //��еDC����
    FramDecode.ucQxBack = ucFramBuff[11];                  //��е��
    FramDecode.ucQxFront = ucFramBuff[12];                 //��е��
    FramDecode.ucDangWei = ucFramBuff[13];                 //��λ
    
    
    
    
    vPca9555_Decode_Uart(&PCA9555_KEY,FramDecode); 
}

/**
 * @brief  ���CRC�Ƿ���ȷ
 * @param  *ucFramBuff  ����ת��֮�������
 * @param  *usCRC  			���ڴ��ݹ�����CRC
 * @return 1-��ȷ 0-����
 * @note   
 */	    uint16_t usTemp;
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC)
{
   
    usTemp = modbus_crc16(ucFramBuff, 16-2);
	 if(usTemp == usCRC) //������Э���ƶ�
	 {
		 return 1;
	 }
	 else return 0;
	
} 

