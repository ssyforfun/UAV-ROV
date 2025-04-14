#ifndef uart_comm_h
#define uart_comm_h

#include "protocol.h"
#include "stm32f4xx_hal.h"
#include "fifo.hpp"
#include "cmsis_os.h"

#define RXBUFFERSIZE PROTOCOL_BUFFER_SIZE
#define TXBUFFERSIZE PROTOCOL_BUFFER_SIZE

void vTaskUartSend(void *pvParameters);
void vTaskUartRecv(void *pvParameters);

// 定义标准串口接口
class UartComm : public CommunicationProtocol
{
public:
    uint32_t rxInterval, txInterval; // 接收/发送的间隔，默认1ms
    virtual void init(int sendSize, UART_HandleTypeDef *huart);

    // ---------- 以下内容基本可以不用修改, 也不会被用到(除了send) -------------------------------------------
public:
    UartComm();
    ~UartComm();
    friend void vTaskUartSend(void *pvParameters);
    friend void vTaskUartRecv(void *pvParameters);

public:
    virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
    virtual uint8_t send(uint8_t *buf, uint32_t length);
    virtual uint8_t send(char *str);
    virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength);

protected:
    bool _isInit; // 是否初始化过，检查是否串口已经可用

protected:
    UART_HandleTypeDef *_hUartHandle;
    bool _isHalfDuplex;
    GPIO_TypeDef* _dirPort;
    uint16_t _dirPin;

public:
    void txDirSet();
    bool rxDirSet();
    void setDirConfig(GPIO_TypeDef* port, uint16_t pin, bool en);
    
protected:
    uint8_t *_sendFifoMemory; // 这个是堆申请 malloc&free
    Fifo<uint8_t> _sendFifo;
    osMutexId _sendUartMutexId;
    osThreadId _sendUart_thread_id;
    osThreadId _recvUart_thread_id;
    int _sendSize;

    uint8_t _rxBuffer[RXBUFFERSIZE];
};

#endif