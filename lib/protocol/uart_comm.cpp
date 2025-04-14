#include "uart_comm.h"
#include "stdio.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "string.h"

static void uart_tx_dma_start(UART_HandleTypeDef *uartP, uint32_t len)
{
    // 清空标志位，不然无法正常使用 HAL_UART_Transmit_DMA 函数
    uartP->gState = HAL_UART_STATE_READY;
    uartP->hdmatx->State = HAL_DMA_STATE_READY;
    uartP->Lock = HAL_UNLOCKED;
    uartP->hdmatx->Lock = HAL_UNLOCKED;

    HAL_UART_Transmit_DMA(uartP, uartP->pTxBuffPtr, len);
}

void vTaskUartSend(void *pvParameters)
{
    UartComm *puc = (UartComm *)pvParameters;
    UART_HandleTypeDef *puh = puc->_hUartHandle;
    bool useHalFuncs = true;
    if (!useHalFuncs)
    {
        puh->hdmatx->Instance->CR &= (~DMA_SxCR_EN); // disable EN to write NDTR
        puh->hdmatx->Instance->NDTR = 0;
        puh->Instance->CR3 |= USART_CR3_DMAT;
        // 配置 DMA 地址
        uart_tx_dma_start(puh, 1); // 发送一个字节是为了让UART做好准备
    }
    int len;
    while (1)
    {
        if ((puh->hdmatx->Instance->NDTR == 0) && (puc->_isInit)) // 代表DMA处于空闲状态
        {
            osMutexWait(puc->_sendUartMutexId, osWaitForever);
            len = puc->_sendFifo.occupiedSize();
            // 代表有数据要发送
            if (len)
            {
                puc->txDirSet();
                len = (len > TXBUFFERSIZE) ? TXBUFFERSIZE : len;
                len = puc->_sendFifo.read(puh->pTxBuffPtr, len);
                if (useHalFuncs)
                {
                    uart_tx_dma_start(puh, len);
                }
                else
                {
                    // 提升了效率，但是直接操控寄存器，移植性差
                    uint32_t *dmaIFCRregaddr = (uint32_t *)(puh->hdmatx->StreamBaseAddress + 8U);
                    *dmaIFCRregaddr = 0x3FU << puh->hdmatx->StreamIndex;
                    puh->hdmatx->Instance->NDTR = len;
                    puh->hdmatx->Instance->CR |= DMA_SxCR_EN;
                }
            }
            osMutexRelease(puc->_sendUartMutexId);
        }

        osDelay(puc->txInterval);
    }
}

void vTaskUartRecv(void *pvParameters)
{
    UartComm *puc = (UartComm *)pvParameters;
    UART_HandleTypeDef *puh = puc->_hUartHandle;
    uint32_t unReadIndex = 0; // 代表一个有效桢的第一个数据的位置

    while (1)
    {
        puc->rxDirSet();

        const int dmaRemCnt = puh->hdmarx->Instance->NDTR;
        const int preIndex = unReadIndex; // 上一次的dma指针位置

        int len = preIndex - dmaRemCnt;
        if (len != 0) // 代表有数据
        {
            // dmaRemCnt是从大到小变的，到0后重置为最大
            int index = RXBUFFERSIZE - preIndex;
            if (index < 0)
                index += RXBUFFERSIZE;

            if (len >= 0) // 如果没有头尾相接
            {
                for (int i = 0; i < len; i++)
                {
                    puc->_rxBuffer[i] = puh->pRxBuffPtr[index++];
                }
            }
            else // 头尾相接，走了一圈
            {
                for (int i = 0; i < preIndex; i++)
                {
                    puc->_rxBuffer[i] = puh->pRxBuffPtr[index++];
                }
                len = RXBUFFERSIZE - dmaRemCnt;
                int bufIndx = preIndex;
                for (int i = 0; i < len; i++)
                {
                    puc->_rxBuffer[bufIndx++] = puh->pRxBuffPtr[i];
                }
            }

            len = preIndex - dmaRemCnt;
            if (len < 0)
                len += RXBUFFERSIZE;

            unReadIndex = dmaRemCnt;
            puc->rxAnalysis(puc->_rxBuffer, len);
        }

        osDelay(puc->rxInterval);
    }
}

UartComm::UartComm()
{
    _sendFifoMemory = NULL;
    _isInit = false;
    rxInterval = 1;
    txInterval = 1;
    _dirPort = NULL;
    _dirPin = 0;
    _isHalfDuplex = false;
}

UartComm::~UartComm()
{
    if (_sendFifoMemory != NULL)
    {
        free(_sendFifoMemory);
    }
}

// 初始化
void UartComm::init(int sendSize, UART_HandleTypeDef *huart)
{
    _sendSize = sendSize;
    _hUartHandle = huart;

    // 申请send fifo内存
    _sendFifoMemory = (uint8_t *)malloc(sendSize * sizeof(uint8_t));
    _sendFifo.init(sendSize, _sendFifoMemory);

    // ------- 启动 串口任务 ---------------
    // 创建 send 任务
    // 创建 互斥锁
    osMutexDef(uartSend);
    _sendUartMutexId = osMutexCreate(osMutex(uartSend)); // 要在任务之前设置，不然可能未初始化就被使用
    osThreadDef(uartSend, vTaskUartSend, osPriority::osPriorityNormal, 0, 256);
    _sendUart_thread_id = osThreadCreate(osThread(uartSend), this);

    // 要确保 串口接收任务的栈大小 足够 这里写256可能会导致栈溢出
    osThreadDef(uartRecv, vTaskUartRecv, osPriority::osPriorityNormal, 0, 512);
    _recvUart_thread_id = osThreadCreate(osThread(uartRecv), this);

    _isInit = true;
}

void UartComm::txDirSet()
{
    if (_isHalfDuplex)
    {
        if (_dirPort != NULL)
        {
            HAL_GPIO_WritePin(_dirPort, _dirPin, GPIO_PIN_SET);
        }
    }
}
bool UartComm::rxDirSet()
{
    bool flag = true;

    if (_isHalfDuplex)
    {
        if (_dirPort != NULL)
        {
            if (_hUartHandle->hdmatx->Instance->NDTR == 0)
            {
                if (HAL_GPIO_ReadPin(_dirPort, _dirPin) == GPIO_PIN_SET)
                {
                    osDelay(1);
                    HAL_GPIO_WritePin(_dirPort, _dirPin, GPIO_PIN_RESET);
                    flag = false;
                }
            }
        }
    }

    return flag;
}

void UartComm::setDirConfig(GPIO_TypeDef *port, uint16_t pin, bool en)
{
    _dirPort = port;
    _dirPin = pin;
    _isHalfDuplex = en;
}

uint8_t UartComm::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
    if (!_isInit)
        return 0;

    osMutexWait(_sendUartMutexId, osWaitForever);
    while (_sendFifo.remainedSize() < (getMiniTxFrameLength() + (int)datalength))
    { // 等待其他任务释放fifo空间
        osMutexRelease(_sendUartMutexId);
        osDelay(1);
        osMutexWait(_sendUartMutexId, osWaitForever);
    }

    int length = datalength;
    const uint8_t *header = data;
    if (_isFrameData) // 如果没有帧结构那么cmd没有意义
    {
        txConstruct(cmd, data, datalength);
        header = _txFrame.getFrame(&length);
    }

    _sendFifo.write(header, length);

    osMutexRelease(_sendUartMutexId);

    return 0;
}

uint8_t UartComm::send(uint8_t *buf, uint32_t length)
{
    if (length >= 1)
        return send(buf, &buf[1], length - 1);
    else
        return 1;
}
uint8_t UartComm::send(char *str)
{
    uint8_t cmd = 's';
    uint32_t datalength = strlen(str);
    uint8_t *data = (uint8_t *)(str);
    return send(&cmd, data, datalength);
}

uint8_t UartComm::send(char cmd, uint8_t *data, uint32_t datalength)
{
    return send((uint8_t *)(&cmd), data, datalength);
}
