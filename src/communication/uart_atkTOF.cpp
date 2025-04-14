#include "uart_atkTOF.h"

static void vTaskTofProcess(void *pvParameters)
{
    UartAtkTof *puartTof = (UartAtkTof *)pvParameters;
    while (1)
    {
        if (puartTof->sendReady)
        {
            puartTof->rxIntervalTick = 0;
            puartTof->sendReady = false;
            uint8_t type = puartTof->_sensorType;
            uint16_t id = puartTof->deviceID[puartTof->sendIndex++];
            if (puartTof->sendIndex >= TOF_NUMBERS)
                puartTof->sendIndex = 0;
            uint8_t iswrite = 0;
            uint8_t reg = TOF_READ_LONGEST_DISTANCE_CMD;
            uint8_t *data = NULL;
            uint8_t databytes = 2;
            puartTof->send(type, id, iswrite, reg, databytes, data);
        }
        else
        {
            if (++puartTof->rxIntervalTick > TOF_RX_INTERVAL) //
            {
                puartTof->rxAnalysis();
                puartTof->sendReady = true;
            } 
            osDelay(1);
        }
    }
}

UartAtkTof::UartAtkTof()
{
}

void UartAtkTof::init(int sendSize, UART_HandleTypeDef *huart)
{
    UartComm::init(sendSize, huart);

    sendReady = true;
    sendIndex = 0;
    _hostHeader = 0x51;
    _clientHeader = 0x55;
    _sensorType = 0x0C;
    rxIntervalTick = 0;
    for (int i = 0; i < TOF_NUMBERS; i++)
        deviceID[i] = 0x2201 + i;
    // 要确保 串口接收任务的栈大小 足够 这里写256可能会导致栈溢出
    osThreadDef(tofProcess, vTaskTofProcess, osPriority::osPriorityNormal, 0, 512);
    _process_task_id = osThreadCreate(osThread(tofProcess), this);
}

uint8_t UartAtkTof::send(uint8_t type, uint16_t id, uint8_t iswrite, uint8_t regaddr, uint8_t datalen, uint8_t *data)
{
    if (!_isInit)
        return 0;

    int len = 9; // ATK-MS53L2M modbus host read command frame length
    if (iswrite == 1)
    {
        len += datalen;
    }

    osMutexWait(_sendUartMutexId, osWaitForever);
    while (_sendFifo.remainedSize() < len)
    { // 等待其他任务释放fifo空间
        osMutexRelease(_sendUartMutexId);
        osDelay(1);
        osMutexWait(_sendUartMutexId, osWaitForever);
    }

    int length;
    uint8_t *header = (uint8_t *)_txFrame.getFrame(&length);
    { // construct
        header[0] = _hostHeader;
        header[1] = type;
        header[2] = ((id >> 8) & 0xFF);
        header[3] = (id & 0xFF);
        header[4] = iswrite;
        header[5] = regaddr;
        header[6] = datalen;
        if (iswrite)
        {
            for (int i = 0; i < datalen; i++)
            {
                header[7 + i] = data[i];
            }
        }
        length = len - 2; // 除去crc sum以外的长度
        uint16_t sum = 0;
        for (int i = 0; i < length; i++)
        {
            sum += header[i];
        }
        header[len - 2] = ((sum >> 8) & 0xFF);
        header[len - 1] = (sum & 0xFF);
    }
    _sendFifo.write(header, len);

    osMutexRelease(_sendUartMutexId);

    return 0;
}

int UartAtkTof::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
    rxIntervalTick = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        if (_rxFrame._index > 255)
            _rxFrame._index = 0;

        _rxFrame._header[_rxFrame._index++] = recvBuf[i];
    }
    return 0;
}

int UartAtkTof::rxAnalysis()
{
    // frame analysis
    int len = _rxFrame._index;
    _rxFrame._index = 0; // reset for next frame reception
    if (len < 8)
        return -1;
    uint8_t *frame = _rxFrame._header;
    if (frame[0] != _clientHeader)
        return -2; // frame wrong
    if (frame[1] != _sensorType)
        return -3; // sensor wrong
    if (frame[4] > 1)
        return -4; // not read or write, fault

    int sum = 0;
    for (int i = 0; i < (len - 2); i++)
    {
        sum += frame[i];
    }
    if (((sum & 0xFF) != frame[len - 1]) || (((sum >> 8) & 0xFF) != frame[len - 2]))
        return -5; // crc sum wrong

    uint8_t datalen = frame[7];
    if ((datalen + 10) != len)
        return -6; // data length wrong

    if (frame[4] == 0)
    {
        uint8_t reg = frame[6];
        switch (reg)
        {
        case TOF_READ_LONGEST_DISTANCE_CMD:
        {
            uint16_t dist = ((frame[8] << 8) + frame[9]);
            int index = sendIndex - 1;
            if (index < 0)
                index += TOF_NUMBERS;
            longestDistance[index] = dist * 1e-3f; // 毫米转米
            break;
        }
        default:
            break;
        }
    }

    return 0;
}

void UartAtkTof::dataAnalysis()
{
    ;
}
