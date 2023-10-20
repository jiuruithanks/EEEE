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
 * @brief ����յ�������֡�Ƿ���ȷ
 *
 * @param buf �յ�������bufferָ�룬ӦΪת�������ݣ������˰�ͷ��CRCУ����
 * @param header �ṹ��ָ�룬���ڱȽ�buffer�е�����֡�Ƿ���header���
 * @param length ���ݵĳ��ȣ������˰�ͷ��CRCУ����
 * @return uint8_t �������
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
 * @brief ��ת���ַ����ݻ�ԭΪδת������
 *
 * @param data_esc ת���ַ���ָ��
 * @param data δת���ַ���ָ��
 * @param length ���յ���ת���ַ����ĳ��ȣ����ػ�ԭ����ַ����ĳ���
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
 * @brief ��buffer�в�������֡ͷ��λ�ã�������һ��ָ��������
 * 
 * @param buf �յ���δת�������bufferָ��
 * @param head ֡ͷָ�����飬���ڴ洢֡ͷ��buffer�еĵ�ַ���������ĳ�������Ϊ֡ͷ�ĸ���+1
 * @param length �յ���δת������ݵĳ��ȣ�Ӧ��С��buf�ĳ��ȡ�
 * @return uint8_t ֡ͷ�ĸ���
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
