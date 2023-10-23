#include "main.h"
#include "pca9539.h"

/*---------报文参数----------------------------------------------*/

typedef struct{
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
} xFramDecode_TypeDef;

void  vStartReceive(void);
    
uint8_t ucCheckCRC(uint8_t *ucFramBuff,uint16_t usCRC);


void vByteSpot( uint8_t *ucBuff );


void vPca9555_Decode_Uart(PCA9555_HandleTypeDef *hdev_key,xFramDecode_TypeDef FramDecode);

void vLightLED(xFramDecode_TypeDef xFramDecode);
