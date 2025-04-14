#ifndef uart_atkTOF_h
#define uart_atkTOF_h

#include "uart_comm.h"

#define TOF_NUMBERS (6)
#define TOF_UPDATE_INTERVAL (10)
#define TOF_READ_LONGEST_DISTANCE_CMD (0x19)
#define TOF_RX_INTERVAL (10)

class UartAtkTof : public UartComm
{
public:
    UartAtkTof();

public:
    virtual void init(int sendSize, UART_HandleTypeDef *huart);
    // 接收数据的数据分析
    virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length); // 数据处理函数
    int rxAnalysis();
    virtual uint8_t send(uint8_t type, uint16_t id, uint8_t iswrite, uint8_t regaddr, uint8_t datalen, uint8_t *data);
    int rxIntervalTick;

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();

public:
    float longestDistance[TOF_NUMBERS]; // unit: m

public:
    uint32_t deviceID[TOF_NUMBERS];
    bool sendReady;
    int sendIndex;
    uint8_t _sensorType;

private:
    osThreadId _process_task_id;
    uint8_t _hostHeader, _clientHeader;
};

#endif
