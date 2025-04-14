#include "uart_modbus.h"
#include "stdio.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "string.h"

static const unsigned char auchCRCHi[] = /* CRC*/
    {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40};

static const unsigned char auchCRCLo[] = /* CRC*/
    {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC,
        0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8,
        0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14,
        0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0,
        0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C,
        0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28,
        0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4,
        0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0,
        0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C,
        0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78,
        0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4,
        0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50,
        0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C,
        0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44,
        0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40};

static unsigned int CRC_Calculate(unsigned char *pdata, unsigned char num) // 传入要校验的数组名及其长度
{
    unsigned char uchCRCHi = 0xFF;
    unsigned char uchCRCLo = 0xFF;
    unsigned char uIndex;
    while (num--)
    {
        uIndex = uchCRCHi ^ *pdata++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo); // CRC校验返回值   // CRCHI 向左移动，就是逆序计算的代表
}

static void uart_tx_dma_start(UART_HandleTypeDef *uartP, uint32_t len)
{
    // 清空标志位，不然无法正常使用 HAL_UART_Transmit_DMA 函数
    uartP->gState = HAL_UART_STATE_READY;
    uartP->hdmatx->State = HAL_DMA_STATE_READY;
    uartP->Lock = HAL_UNLOCKED;
    uartP->hdmatx->Lock = HAL_UNLOCKED;
    HAL_UART_Transmit_DMA(uartP, uartP->pTxBuffPtr, len);
}
// void vTaskModbusSend(void *pvParameters)
// {
//     UartComm *puc = (UartComm *)pvParameters;
//     UART_HandleTypeDef *puh = puc->_hUartHandle;
//     int len;
//     while (1)
//     {
//         if ((puh->hdmatx->Instance->NDTR == 0) && (puc->_isInit)) // 代表DMA处于空闲状态
//         {
//             osMutexWait(puc->_sendUartMutexId, osWaitForever);
//             len = puc->_sendFifo.occupiedSize();
//             // 代表有数据要发送
//             if (len)
//             {
//                 len = (len > TXBUFFERSIZE) ? TXBUFFERSIZE : len;
//                 len = puc->_sendFifo.read(puh->pTxBuffPtr, len);
//                 // uart_tx_dma_start(puh, len);
//                 puh->hdmatx->Instance->NDTR = len;
//             }
//             osMutexRelease(puc->_sendUartMutexId);
//         }
//         osDelay(puc->txInterval);
//     }
// }

void vTaskModbusRecv(void *pvParameters)
{
    UartModbus *pum = (UartModbus *)pvParameters;
    UART_HandleTypeDef *puh = pum->_hUartHandle;
    uint32_t unReadIndex = 0; // 代表一个有效桢的第一个数据的位置

    while (1)
    {
        const int dmaRemCnt = puh->hdmarx->Instance->NDTR;
        const int preIndex = unReadIndex; // 上一次的dma指针位置

        int len = preIndex - dmaRemCnt;
        if (len != 0) // 代表有数据
        {
            pum->_rxTimeTick = 0; // 超时计数清零

            // dmaRemCnt是从大到小变的，到0后重置为最大
            int index = RXBUFFERSIZE - preIndex;
            if (index < 0)
                index += RXBUFFERSIZE;

            if (len >= 0) // 如果没有头尾相接
            {
                for (int i = 0; i < len; i++)
                {
                    pum->_rxBuffer[i] = puh->pRxBuffPtr[index++];
                }
            }
            else // 头尾相接，走了一圈
            {
                for (int i = 0; i < preIndex; i++)
                {
                    pum->_rxBuffer[i] = puh->pRxBuffPtr[index++];
                }
                len = RXBUFFERSIZE - dmaRemCnt;
                int bufIndx = preIndex;
                for (int i = 0; i < len; i++)
                {
                    pum->_rxBuffer[bufIndx++] = puh->pRxBuffPtr[i];
                }
            }

            len = preIndex - dmaRemCnt;
            if (len < 0)
                len += RXBUFFERSIZE;

            unReadIndex = dmaRemCnt;
            pum->rxAnalysis(pum->_rxBuffer, len);
        }
        else
        {
            if (pum->_rxTimeTick++ >= pum->_rxTimeout)
            {
                pum->_rxTimeTick = 0;
                pum->rxAnalysis();
            }
        }

        osDelay(pum->rxInterval);
    }
}

UartModbus::UartModbus()
{
    _isInit = false;
    rxInterval = 1;
    _rxTimeout = 2; //
    _rxTimeTick = 0;
    _id = 0x01;
    _readByteLength = 0;
    _readStartAddr = 0;
}

// 初始化
void UartModbus::init(UART_HandleTypeDef *huart)
{
    _rxFrame.init(_rxFrameBuffer, 1, 1, 0, 2);
    _txFrame.init(_txFrameBuffer, 1, 1, 0, 2);
    setID(_id);

    _hUartHandle = huart;
    // ------- 启动 串口任务 ---------------
    osThreadDef(uartRecv, vTaskModbusRecv, osPriority::osPriorityNormal, 0, 256);
    _recvUart_thread_id = osThreadCreate(osThread(uartRecv), this);

    _isInit = true;
}

void UartModbus::setID(uint8_t id)
{
    _txFrame.setHeader(&id);
    _rxFrame.setHeader(&id);
}

int UartModbus::rxAnalysis(const uint8_t *const recvBuf, uint32_t length)
{
    uint8_t dataTmp;

    for (uint32_t i = 0; i < length; i++)
    {
        dataTmp = recvBuf[i];

        // 帧头 header --> modbus地址
        if (_rxFrame._index < _rxFrame._headerEnd)
        { // 处于桢头位置，需要判断桢头是否正确
            if (dataTmp != _rxFrame._header[_rxFrame._index])
            { // 如果桢头对应不上，从当前字节开始认为下一个桢的起点
                if (dataTmp != _rxFrame._header[0])
                {
                    _rxFrame._index = 0;
                    continue;
                }
                else
                {
                    _rxFrame._index++;
                    continue;
                }
            }
            else
            {
                _rxFrame._index++;
                continue;
            }
        }

        // 数据
        if (_rxFrame._index < RXBUFFERSIZE)
        {
            _rxFrame._header[_rxFrame._index++] = dataTmp;
            continue;
        }

        // // 命令 cmd --> modbus code
        // if (_rxFrame._index < _rxFrame._cmdEnd)
        // {
        //     _rxFrame._header[_rxFrame._index++] = dataTmp;
        //     _rxCode = dataTmp;
        //     _rxFrame._dataNumEnd = _rxFrame._cmdEnd;
        //     if (_rxFrame._index == _rxFrame._cmdEnd)
        //     {
        //         switch (_rxCode)
        //         {
        //         case 0x03: // 从机返回的是 发送要求的长度
        //             _rxFrame._dataLen = _readByteLength;
        //             _rxFrame._dataEnd = _rxFrame._dataNumEnd + _rxFrame._dataLen;
        //             _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
        //             _rxFrame._length = _rxFrame._checkEnd;
        //             break;
        //         case 0x06:                 // 从机返回的是 2个字节 (一个起始地址, 一个数据)
        //             _rxFrame._dataLen = 4; // int16_t
        //             _rxFrame._dataEnd = _rxFrame._dataNumEnd + _rxFrame._dataLen;
        //             _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
        //             _rxFrame._length = _rxFrame._checkEnd;
        //             break;
        //         case 0x10:                 // 从机返回的是 4个字节 (一个起始地址, 一个长度数据)
        //             _rxFrame._dataLen = 4; // int16_t
        //             _rxFrame._dataEnd = _rxFrame._dataNumEnd + _rxFrame._dataLen;
        //             _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
        //             _rxFrame._length = _rxFrame._checkEnd;
        //             break;
        //         default:
        //             break;
        //         }
        //     }
        //     continue;
        // }

        // // 数据
        // if (_rxFrame._index < _rxFrame._dataEnd)
        // {
        //     _rxFrame._header[_rxFrame._index++] = dataTmp;
        //     continue;
        // }

        // // 校验
        // if (_rxFrame._index < _rxFrame._checkEnd)
        // {
        //     _rxFrame._header[_rxFrame._index++] = dataTmp;
        //     if (_rxFrame._index == _rxFrame._length)
        //     {
        //         if (checkAnalysis() == 0) // 校验OK
        //             dataAnalysis();
        //         _rxFrame._index = 0;
        //     }
        //     continue;
        // }
    }

    return 0;
}

int UartModbus::rxAnalysis()
{
    _rxFrame._length = _rxFrame._index;
    if (_rxFrame._length < 4)
    {
        _rxFrame._index = 0;
        return 0;
    }

    _rxFrame._dataLen = _rxFrame._length - 4; // 4: modbusID(1) + code(1) + crcCheck(2)
    _rxFrame._dataEnd = _rxFrame._cmdEnd + _rxFrame._dataLen;
    _rxFrame._checkEnd = _rxFrame._dataEnd + _rxFrame._checkLen;
    _rxCode = _rxFrame._header[1];
    // switch (_rxCode)
    // {
    // case 0x03: // 从机返回的是 发送要求的长度
    //     break;
    // case 0x06: // 从机返回的是 2个字节 (一个起始地址, 一个数据)
    //     break;
    // case 0x10: // 从机返回的是 4个字节 (一个起始地址, 一个长度数据)
    //     break;
    // default:
    //     break;
    // }

    // 校验
    if (checkAnalysis() == 0) // 校验OK
        dataAnalysis();

    _rxFrame._index = 0;
    return 0;
}

// 返回0代表成功，其它代表失败
int UartModbus::checkAnalysis()
{
    uint32_t crccalc = CRC_Calculate(_rxFrame._header, _rxFrame._length - 2);
    uint8_t crch = ((crccalc >> 8) & 0xFF);
    uint8_t crcl = (crccalc & 0xFF);
    uint8_t crcBh = _rxFrame._header[_rxFrame._length - 2];
    uint8_t crcBl = _rxFrame._header[_rxFrame._length - 1];
    if ((crch == crcBh) && (crcl == crcBl))
        // if ((((crccalc >> 8) & 0xFF) == _rxFrame._header[_rxFrame._length - 2]) &&
        //     ((crccalc & 0xFF) == _rxFrame._header[_rxFrame._length - 1]))
        return 0;
    else
        return -1;
}

uint8_t UartModbus::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
    return 0;
}

int UartModbus::send(uint8_t code, uint16_t dataByteLen, uint8_t *value)
{
    if (!_isInit)
        return -1;

    while (1)
    {
        if (_hUartHandle->hdmatx->Instance->NDTR == 0) // 代表DMA处于空闲状态
        {
            if (code == 0x03)
            {
                if (dataByteLen != 4)
                    return -2;
                _readByteLength = ((value[2] << 8) + value[3]) * 2; //
                _readStartAddr = ((value[0] << 8) + value[1]);
            }
            else if (code == 0x06)
            {
                if (dataByteLen != 4)
                    return -2;
            }

            uint8_t *buf = _hUartHandle->pTxBuffPtr;
            buf[0] = _id;
            buf[1] = code;
            for (int i = 0; i < dataByteLen; i++)
            {
                buf[2 + i] = value[i];
            }

            uint32_t crc = CRC_Calculate(buf, 2 + dataByteLen);
            buf[2 + dataByteLen] = ((crc >> 8) & 0xFF);
            buf[3 + dataByteLen] = (crc & 0xFF);

            uart_tx_dma_start(_hUartHandle, 4 + dataByteLen);
            //_hUartHandle->hdmatx->Instance->NDTR = 4 + dataByteLen;
            break;
        }
    }
    return 0;
}

// 0x03，读寄存器
int UartModbus::readRegister(uint16_t addr, int length)
{
    uint8_t buf[4];
    buf[0] = ((addr >> 8) & 0xFF);
    buf[1] = (addr & 0xFF);
    buf[2] = ((length >> 8) & 0xFF);
    buf[3] = (length & 0xFF);
    send(0x03, 4, buf);
    return 0;
}

// 0x06，写单个寄存器
int UartModbus::writeRegister(uint16_t addr, uint16_t value)
{
    uint8_t buf[4];
    buf[0] = ((addr >> 8) & 0xFF);
    buf[1] = (addr & 0xFF);
    buf[2] = ((value >> 8) & 0xFF);
    buf[3] = (value & 0xFF);
    send(0x06, 4, buf);
    return 0;
}

// 0x10，写多个寄存器
int UartModbus::writeRegisters(uint16_t addr, uint16_t *value, int length)
{
    uint8_t buf[length * 2 + 2];
    buf[0] = ((addr >> 8) & 0xFF);
    buf[1] = (addr & 0xFF);
    buf[2] = ((length >> 8) & 0xFF);
    buf[3] = (length & 0xFF);
    for (int i = 0; i < length; i++)
    {
        buf[4 + 2 * i] = ((value[i] >> 8) & 0xFF);
        buf[5 + 2 * i] = (value[i] & 0xFF);
    }
    send(0x10, length * 2 + 4, buf);
    return 0;
}
