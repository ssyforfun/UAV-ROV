#ifndef uart_modbus_h
#define uart_modbus_h

#include "protocol.h"
#include "stm32f4xx_hal.h"
#include "fifo.hpp"
#include "cmsis_os.h"

#define RXBUFFERSIZE PROTOCOL_BUFFER_SIZE
#define TXBUFFERSIZE PROTOCOL_BUFFER_SIZE

void vTaskModbusSend(void *pvParameters);
void vTaskModbusRecv(void *pvParameters);

// 定义自定义的modbus主机
class UartModbus : public CommunicationProtocol
{
public:
    uint32_t rxInterval; // 接收的间隔，默认1ms
    void init(UART_HandleTypeDef *huart);

public:
    virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);

    // ---------- 以下内容基本可以不用修改, 也不会被用到(除了send) -------------------------------------------
public:
    UartModbus();
    friend void vTaskModbusSend(void *pvParameters);
    friend void vTaskModbusRecv(void *pvParameters);

public:
    virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length);
    int rxAnalysis();
    int send(uint8_t code, uint16_t dataByteLen, uint8_t *value);
    virtual int readRegister(uint16_t addr, int length);                   // 0x03寄存器
    virtual int writeRegister(uint16_t addr, uint16_t value);               // 0x06寄存器
    virtual int writeRegisters(uint16_t addr, uint16_t *value, int length); // 0x10寄存器

public:
    void setID(uint8_t id);
    uint32_t _readByteLength; // 当发送03寄存器时，要得到的返回字节数据长度。在readRegister/send里要设置更新
    uint32_t _readStartAddr;  // 当发送03寄存器时，起始寄存器地址

private:
    bool _isInit; // 是否初始化过，检查是否串口已经可用
    uint8_t _id;  // modbus地址

public:
    UART_HandleTypeDef *_hUartHandle;

private:
    osThreadId _recvUart_thread_id;
    //UART_HandleTypeDef *_hUartHandle;
    uint8_t _rxBuffer[RXBUFFERSIZE];
    uint8_t _txBuffer[TXBUFFERSIZE];

    virtual int checkAnalysis(); // 校验函数 重写

    int _rxTimeout;
    int _rxTimeTick;
    uint8_t _rxCode; // 存储接收信息的 modbus code
};

#endif