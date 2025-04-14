#ifndef comm_recv_h
#define comm_recv_h

#include "stddef.h"
#include "stdint.h"

class CommRecv
{
public:
    CommRecv();

public:
    virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength) = 0;
    virtual uint8_t send(uint8_t *buf, uint32_t length) = 0;
    virtual uint8_t send(char *str) = 0;
    virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength) = 0;

public:
    void msgAnalysis(uint8_t cmd, uint8_t *data, uint8_t datalen);

private:
    uint8_t *_dataPtr;
    int _datalength;
    uint8_t _cmd;

    // ----  接收数据的分析 --------------------------------------------
public:
    void data_connect();
    void data_aquisition();
    void data_controlmode();

protected:
    enum COMM_COMMAND
    {
        NO_CMD = 0,
        ANGLE_CMD = 'a',
        CONTROLMODE_CMD = 'c',
        AQUISITION_CMD = 'f',
        READ_CMD = 'r',
        STRING_CMD = 's',
        SHELL_CMD = 'x',
        TEST_CMD = 'z',
    };
};

#endif
