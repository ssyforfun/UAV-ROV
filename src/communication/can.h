#ifndef can_h
#define can_h

#include <stm32f4xx_hal.h>
#include "comm_recv.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "fifo.hpp"

// STID 定义 [10:0]
// 前两位 reserved, 必须是00
// 8-5: 代表设备类型，或称设备ID
// 4-0: 代表命令类型，或其它
//

void can_init();
void can1RxCallback();
void can2RxCallback();
void can_test(uint8_t id);

class CanComm : public CommunicationProtocol, public CommRecv
{
public:
    CanComm();
    void init();
    void rxProcess();
    int sendDataStdID(uint32_t stdId, uint8_t *data, int datalen);
    int sendDataExtID(uint32_t extId, uint8_t *data, int datalen);
    void rxDataAnalysis();

public:
    void setHandle(CAN_HandleTypeDef *can_handle);

public:
    uint32_t txCount;
    uint32_t rxCount;

public:
    CAN_HandleTypeDef CanHandle;

public:
    void rxSignalSet() { osSignalSet(can_rx_thread_id, 1); }

private:
    osThreadId can_rx_thread_id, can_tx_thread_id;
    
public:
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t TxData[8];
    uint8_t RxData[8];

public:
    osMutexId canComputerTxMutexId;
    uint8_t sendFifoMemory[256];
    Fifo<uint8_t> sendFifo;

public:
    uint8_t rxFifoMemory[256];
    Fifo<uint8_t> rxFifo;

public:
    virtual uint8_t send(uint8_t *buf, uint32_t length);
    virtual uint8_t send(char *str);
    virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
    virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength);

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();
};

extern CanComm canComm[2];
#endif
