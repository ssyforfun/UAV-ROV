/************************************************************
 * FileName:        mySocket.cpp
 * Description:     socket connection method
 *                  ESP32
 * Auther:          Jinsheng
 * CreateDate:      2021-06-01
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include <motion.h>
#include "fifo.hpp"
#include "string.h"

#define MOTIONDATA_SIZE (2048)
Fifo<MotionData_t> motionFifo;
MotionData_t motionData[MOTIONDATA_SIZE];

Motion::Motion()
{
    init();
}

Motion::~Motion()
{
}

void Motion::init(void)
{
    motionFifo.init(MOTIONDATA_SIZE, motionData);
}

int Motion::write(const uint8_t *data, int data_len)
{
    int len = data_len / sizeof(MotionData_t);
    if ((int)(len * sizeof(MotionData_t)) == data_len)
    {
        // 这种强制转换需要内存地址对齐来辅助，不然就会出现hard fault故障
        // motionFifo.write((MotionData_t *)data, len);
        if (motionFifo.writeByBytes(data, len) == data_len)
            return 1;
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }
}

// 返回的是字节长度，-1代表出错
int Motion::read(uint8_t *buf, int len)
{
    MotionData_t mbuf[len];
    int datalen;
    datalen = motionFifo.read(mbuf, len);
    if (datalen > 0)
    {
        datalen *= sizeof(MotionData_t);
        uint8_t *src = (uint8_t *)mbuf;
        uint8_t *des = (uint8_t *)buf;
        for (int i = 0; i < datalen; i++)
        {
            *des++ = *src++;
        }
        return datalen;
    }
    else
    {
        return -1;
    }
}

// 返回的是MotionData_t的个数
int Motion::read(MotionData_t *buf, int len)
{
    return motionFifo.read(buf, len);
}

// 返回的是字节长度，-1代表出错
int Motion::peek(uint8_t *buf, int len, int offset)
{
    MotionData_t mbuf[len];
    int datalen;
    datalen = peek(mbuf, len, offset);
    if (datalen > 0)
    {
        datalen *= sizeof(MotionData_t);
        uint8_t *src = (uint8_t *)mbuf;
        uint8_t *des = (uint8_t *)buf;
        for (int i = 0; i < datalen; i++)
        {
            *des++ = *src++;
        }
        return datalen;
    }
    else
    {
        return -1;
    }
}

// 返回的是MotionData_t的个数
int Motion::peek(MotionData_t *des, int len, int offset)
{
    int i;
    const MotionData_t *mfbuf;
    for (i = 0; i < len; i++)
    {
        mfbuf = (const MotionData_t *)motionFifo.peek(offset + i);
        if (mfbuf == NULL)
            return -1;
        *des++ = *mfbuf;
    }
    return i;
}

int Motion::peekLatest(uint8_t *buf)
{
    return peek(buf, 1, 0);
}

int Motion::peekLast(uint8_t *buf)
{
    return peek(buf, 1, motionFifo.occupiedSize() - 1);
}

int Motion::peekLatest(MotionData_t *buf)
{
    if (peek(buf, 1, 0) != 1)
    {
        memset(buf, 0, sizeof(MotionData_t));
        return -1;
    }
    return 1;
}

int Motion::peekLast(MotionData_t *buf)
{
    if (peek(buf, 1, motionFifo.occupiedSize() - 1) != 1)
    {
        memset(buf, 0, sizeof(MotionData_t));
        return -1;
    }
    return 1;
}

int Motion::pop(int len)
{
    return motionFifo.pop(len);
}

int Motion::occupy(void)
{
    return motionFifo.occupiedSize();
}

int Motion::reset(void)
{
    return motionFifo.reset();
}

Motion motion;
