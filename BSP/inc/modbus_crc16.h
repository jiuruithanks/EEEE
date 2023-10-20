#ifndef MODBUS_CRC16_H
#define MODBUS_CRC16_H

//#include "type.h"




/******************************************************************************
  Name:         modbus_crc16
  Parameters:   address of data, length of data
  Return Value: CRC value
  Descriptions: modbus crc16 calculation
 ******************************************************************************/
unsigned short int modbus_crc16(unsigned char *puchMsg,unsigned char usDataLen);





#endif /* MODBUS_CRC16_H */





