#include "main.h"
#include "pca9539.h"

/*---------���Ĳ���----------------------------------------------*/

typedef struct{
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
} xFramDecode_TypeDef;

void  vStartReceive(void);
    
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC);


void vByteSpot( uint8_t *ucBuff );


void vPca9555_Decode_Uart(PCA9555_HandleTypeDef *hdev_key,xFramDecode_TypeDef FramDecode);

void vLightLED(xFramDecode_TypeDef xFramDecode);
