#include "uart_computer.h"
#include "main.h"

UartComputer::UartComputer()
{
}

uint8_t UartComputer::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
	return UartComm::send(cmd, data, datalength);
}
uint8_t UartComputer::send(uint8_t *buf, uint32_t length)
{
	return UartComm::send(buf, length);
}
uint8_t UartComputer::send(char *str)
{
	return UartComm::send(str);
}
uint8_t UartComputer::send(char cmd, uint8_t *data, uint32_t datalength)
{
	return UartComm::send(cmd, data, datalength);
}

void UartComputer::dataAnalysis()
{
	userComm = this;
    if (_isFrameData)
        msgAnalysis(_rxFrame._cmdPtr[0], _rxFrame._dataPtr, _rxFrame._dataLen);
    else
        msgAnalysis(SHELL_CMD, _rxFrame._header, _rxFrame._dataLen);
}
