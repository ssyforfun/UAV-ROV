#ifndef uart_computer_h
#define uart_computer_h

#include "comm_recv.h"
#include "uart_comm.h"

class UartComputer : public UartComm, public CommRecv
{
public:
    UartComputer();

public:
    virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
    virtual uint8_t send(uint8_t *buf, uint32_t length);
    virtual uint8_t send(char *str);
    virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength);

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();
};

#endif