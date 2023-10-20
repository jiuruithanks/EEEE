/**
 * @file handle.c
 * @author Zhao Haofei
 * @brief Provide handle a data frame check function.
 * @version 1.0
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "data_format.h"


/**
 * @brief 检查收到的数据帧是否正确
 *
 * @param buf 收到的数据buffer指针，应为转义后的数据，包含了包头和CRC校验码
 * @param header 结构体指针，用于比较buffer中的数据帧是否与header相符
 * @param length 数据的长度，包含了包头和CRC校验码
 * @return uint8_t 错误代码
 */
uint8_t FrameCheck(uint8_t *buf, FrameHeader *header, uint8_t length)
{
    uint8_t err = 0;
    uint16_t crc = 0;

    /* Check header */
    if (buf[2] != header->sender)
    {
        err |= FRAME_SENDER_ERROR;
    }
    if (buf[3] != header->receiver)
    {
        err |= FRAME_RECEIVER_ERROR;
    }
    if (buf[4] != header->type)
    {
        err |= FRAME_TYPE_ERROR;
    }

    /* Check data length if valid */
    if (length != (buf[1] + 2))
    {
        err |= FRAME_LENGTH_ERROR;
        return err;
    }

    /* CRC check */
    crc = modbus_crc16(buf, buf[1]);
    if (crc != ((buf[buf[1] + 1] << 8) | buf[buf[1]]))
    {
        err |= FRAME_CRC_ERROR;
    }

    return err;
}

/**
 * @brief 将转义字符数据还原为未转义数据
 *
 * @param data_esc 转义字符串指针
 * @param data 未转义字符串指针
 * @param length 接收到的转义字符串的长度，返回还原后的字符串的长度
 */
void DeleteEsc(uint8_t *data_esc, uint8_t *data, uint8_t *length)
{
    uint8_t i = 0, j = 0;

    for (i = 0, j = 0; i < *length; i++, j++)
    {
        if (data_esc[i] == 0x1b)
        {
            if (data_esc[i + 1] == 0x00)
            {
                data[j] = 0x1b;
            }
            else if (data_esc[i + 1] == 0xe7)
            {
                data[j] = 0x02;
            }
            i++;
        }
        else
        {
            data[j] = data_esc[i];
        }
    }

    *length = j;
}

/**
 * @brief 从buffer中查找所有帧头的位置，保存在一个指针数组中
 * 
 * @param buf 收到的未转义的数据buffer指针
 * @param head 帧头指针数组，用于存储帧头在buffer中的地址，这个数组的长度至少为帧头的个数+1
 * @param length 收到的未转义的数据的长度，应当小于buf的长度。
 * @return uint8_t 帧头的个数
 */
uint8_t FindHead(uint8_t *buf, uint8_t *head[], uint8_t *length)
{
    uint8_t num = 0;

    for (uint8_t i = 0; i < *length; i++)
    {
        if (buf[i] == 0x02)
        {
            head[num] = &buf[i];
            num++;
        }
    }

    head[num] = &buf[*length];
    return num;
}
