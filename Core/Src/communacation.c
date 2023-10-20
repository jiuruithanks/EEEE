#include "communacation.h"
#include <string.h>
#include "modbus_crc16.h"

uint8_t ucSendDataBuff[10]={0};

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/*-------���ڲ���-----------------------------------------------*/

#define BUFFER_SIZE     32                    //��Э�����
uint8_t ucDMAReceiveBuff[BUFFER_SIZE] = {0};  //DMA����������

uint8_t ucFramBuff[BUFFER_SIZE];//���������


/*---------���Ĳ���----------------------------------------------*/
uint8_t  ucFramHead;                //��ͷ
uint8_t  ucFramLength;              //������
uint8_t  ucFramRecever;             //������
uint8_t  ucFramType;                //������
uint8_t  ucEscKey;                  //�˳�����
uint8_t  ucOptKey;                  //��������
uint8_t  ucEnterKey;                //���밴��
uint8_t  ucDsDcOpenOrClose;         //��˿DC����
uint8_t  ucDsBack;                  //��˿��
uint8_t  ucDsFront;                 //��˿��
uint8_t  usQxDcOpenOrClose;         //��еDC����
uint8_t  ucQxBack;                  //��е��
uint8_t  ucQxFront;                 //��е��
uint8_t  ucDangWei;                 //��λ
uint16_t usCRC;


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


/**
 * @brief ����ַ����н���
 * @param  *ucUNTranslateBuff  ���������
 * @param  ucDataLength        ����������ֽڳ���
 * @return none
 * @note   
 */	
void vByteSpot( uint8_t *ucBuff )
{
    uint8_t ucFramBuff[19];   //ת��������-������Э��̶�

    /*1.CRCУ��*/
    usCRC =  ((uint16_t)ucFramBuff[15] <<8) | ucFramBuff[14];        

    if(ucCheckCRC(ucFramBuff,usCRC)==0){return;}

    /*2.���ֽڽ���*/
    ucFramHead = ucFramBuff[0];    //��ͷ
    ucFramLength = ucFramBuff[1];  //������
    ucFramRecever = ucFramBuff[2]; //������
    ucFramType = ucFramBuff[3];    //������
    
    ucEscKey = ucFramBuff[4];                   //�˳�����
    ucOptKey = ucFramBuff[5];                   //��������
    ucEnterKey = ucFramBuff[6];                 //���밴��
    ucDsDcOpenOrClose = ucFramBuff[7];          //��˿DC����
    ucDsBack = ucFramBuff[8];                   //��˿��
    ucDsFront = ucFramBuff[9];                  //��˿��
    usQxDcOpenOrClose = ucFramBuff[10];         //��еDC����
    ucQxBack = ucFramBuff[11];                  //��е��
    ucQxFront = ucFramBuff[12];                 //��е��
    ucDangWei = ucFramBuff[13];                 //��λ
    
    
    


}

/**
 * @brief  ���CRC�Ƿ���ȷ
 * @param  *ucFramBuff  ����ת��֮�������
 * @param  *usCRC  			���ڴ��ݹ�����CRC
 * @return 1-��ȷ 0-����
 * @note   
 */	
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC)
{
	 if(modbus_crc16(ucFramBuff, 17) == usCRC) //������Э���ƶ�
	 {
		 return 1;
	 }
	 else return 0;
	
} 

