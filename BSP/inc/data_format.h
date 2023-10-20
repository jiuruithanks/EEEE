#ifndef _DATA_FORMAT_HEADER_
#define _DATA_FORMAT_HEADER_

#include <stdint.h>
#include "modbus_crc16.h"

typedef struct _FrameHeader
{
    uint8_t len;
    uint8_t sender;
    uint8_t receiver;
    uint8_t type;
}FrameHeader;


#define DEVICE_HANDLE   0x01
#define DEVICE_HOST     0x02
#define DEVICE_MOVER    0x04
#define DEVICE_OPERATOR 0x08

// #define FRAME_HEADER              0x02
// #define FRAME_SENDER              0x01
// #define FRAME_RECEIVER            0x0C
// #define FRAME_TYPE                0x01

#define FRAME_LENGTH_ERROR   (1 << 1)
#define FRAME_SENDER_ERROR   (1 << 2)
#define FRAME_RECEIVER_ERROR (1 << 3)
#define FRAME_TYPE_ERROR     (1 << 4)
#define FRAME_CRC_ERROR      (1 << 5)

void DeleteEsc(uint8_t *data_esc, uint8_t *data, uint8_t *length);
uint8_t FindHead(uint8_t *buf, uint8_t *head[], uint8_t *length);
uint8_t FrameCheck(uint8_t *buf, FrameHeader *header, uint8_t length);

#endif
