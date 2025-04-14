#ifndef uart_miniSVP_h
#define uart_miniSVP_h

#include "uart_comm.h"

class UartMiniSVP : public UartComm
{
public:
    UartMiniSVP();

public:
    // 接收数据的数据分析
	virtual int rxAnalysis(const uint8_t *const recvBuf, uint32_t length); // 数据处理函数

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();

private:
    void svpData();

public:
    float pressure, temperature, conductivity;
    float string2float(uint8_t *str, int start, int end, bool *valid=NULL);
};

#endif