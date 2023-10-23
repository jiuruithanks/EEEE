#include "main.h"
#include "led_key_tasks.h"
#include "tmc5160.h"
#include "bsp.h"
#include "communacation.h"


extern xFramDecode_TypeDef LastFramDecode;
static uint8_t ucKeyValuePre[16] = {0};
    
void vPca9555_Decode_Uart(PCA9555_HandleTypeDef *hdev_key,xFramDecode_TypeDef FramDecode)
{
    uint8_t ucKeyValue[16] = {0};
    

    
    /*1.新按键值填充*/
    hdev_key->key[0] = FramDecode.ucDsDcOpenOrClose;
    hdev_key->key[1] = FramDecode.ucDsBack;
    hdev_key->key[2] = FramDecode.ucDsFront;
    hdev_key->key[3] = FramDecode.ucQxBack;
    hdev_key->key[4] = FramDecode.ucQxFront;
    hdev_key->key[5] = FramDecode.usQxDcOpenOrClose;
    hdev_key->key[6] = FramDecode.ucOptKey;
    hdev_key->rotate = FramDecode.ucDangWei;
    
    
    /*2.作出相应action*/
	Update_Key_status(hdev_key, 0);
    
    /*3.给出LED控制*/
    vLightLED(FramDecode);
    
    /*将此次新值作为旧值保存*/
    hdev_key->key_pre[0] = FramDecode.ucDsDcOpenOrClose;
    hdev_key->key_pre[1] = FramDecode.ucDsBack;
    hdev_key->key_pre[2] = FramDecode.ucDsFront;
    hdev_key->key_pre[3] = FramDecode.ucQxBack;
    hdev_key->key_pre[4] = FramDecode.ucQxFront;
    hdev_key->key_pre[5] = FramDecode.usQxDcOpenOrClose;
    hdev_key->key_pre[6] = FramDecode.ucOptKey;
    
    LastFramDecode.ucDsDcOpenOrClose = FramDecode.ucDsDcOpenOrClose;
    LastFramDecode.ucDsBack = FramDecode.ucDsBack;
    LastFramDecode.ucDsFront = FramDecode.ucDsFront;
    LastFramDecode.ucQxBack = FramDecode.ucQxBack;
    LastFramDecode.ucQxFront = FramDecode.ucQxFront;
    LastFramDecode.usQxDcOpenOrClose = FramDecode.usQxDcOpenOrClose;
    LastFramDecode.ucOptKey = FramDecode.ucOptKey;
    
}


