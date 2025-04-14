
#include "usb_computer.h"
#include "fifo.hpp"
#include "main.h"

USBComm usbComm;
static uint8_t sendFifoMemory[TXBUFFERSIZE];
static Fifo<uint8_t> sendFifo(TXBUFFERSIZE, sendFifoMemory); // 10k bytes

osSemaphoreId sem_usb_irq;
osMutexId usbTxMutexId;
osThreadId usb_irq_thread_id;
osThreadId sendUsb_thread_id;

extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static __IO bool sendValid = false;

void usbRxCallback(const uint8_t *const buf, uint32_t length);
void vThreadUsbIRQHandler(void *pvParameters);
static void vTaskUsbSend(void *pvParameters);

void usb_init(void)
{
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
    osSemaphoreWait(sem_usb_irq, 0);

    osMutexDef(usbSend);
    usbTxMutexId = osMutexCreate(osMutex(usbSend)); // 要在任务之前设置，不然可能未初始化就被使用

    osThreadDef(task_usb_irq, vThreadUsbIRQHandler, osPriority::osPriorityAboveNormal, 0, 1024);
    usb_irq_thread_id = osThreadCreate(osThread(task_usb_irq), NULL);

    osThreadDef(usbSend, vTaskUsbSend, osPriority::osPriorityNormal, 0, 256);
    sendUsb_thread_id = osThreadCreate(osThread(usbSend), NULL);

    USB_PP_init(ISR_Priority::USB_Prior, (FuncRxCallBack)usbRxCallback);
}

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void vThreadUsbIRQHandler(void *pvParameters)
{
    while (true)
    {
        // Wait for signalling from USB interrupt (OTG_FS_IRQHandler)
        osStatus semaphore_status = osSemaphoreWait(sem_usb_irq, osWaitForever);
        if (semaphore_status == osOK)
        {
            // We have a new incoming USB transmission: handle it
            HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
            // Let the irq (OTG_FS_IRQHandler) fire again.
            HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        }
    }
    // vTaskDelete(NULL);
}

void usbRxCallback(const uint8_t *const buf, uint32_t length)
{
    // USB_txBlock((uint8_t *)buf, length);
    //userComm = &usbComm; // 在类里实现了
    usbComm.rxAnalysis(buf, length);
}

static void vTaskUsbSend(void *pvParameters)
{
    int len;
    int frameMinimumLen = usbComm.getMiniTxFrameLength();
    sendValid = true;
    while (1)
    {
        osMutexWait(usbTxMutexId, osWaitForever);
        len = sendFifo.occupiedSize();
        // 代表有数据要发送
        if (len >= frameMinimumLen)
        {
            len = (len > TXBUFFERSIZE) ? TXBUFFERSIZE : len;
            len = sendFifo.read(UserTxBufferFS, len);
            USB_txBlock(UserTxBufferFS, len);
            osMutexRelease(usbTxMutexId);
        }
        else
        {
            osMutexRelease(usbTxMutexId);
            vTaskDelayMs(1);
        }        
    }

    // 如果任务不是永久性的需要调用 vTaskDelete 删除任务
    // vTaskDelete(NULL);
}

USBComm::USBComm()
{
}

uint8_t USBComm::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
    while (!sendValid)
    { // 代表send口还没有准备好
        delay(1);
    }
    osMutexWait(usbTxMutexId, osWaitForever);
    while (sendFifo.remainedSize() < (getMiniTxFrameLength() + (int)datalength))
    { // 等待其他任务释放fifo空间
        osMutexRelease(usbTxMutexId);
        osDelay(1);
        osMutexWait(usbTxMutexId, osWaitForever);
    }

    int length = datalength;
    const uint8_t *header = data;
    if (_isFrameData) // 如果没有帧结构那么cmd没有意义
    {
        txConstruct(cmd, data, datalength);
        header = _txFrame.getFrame(&length);
    }

    sendFifo.write(header, length);
    
    osMutexRelease(usbTxMutexId);

    return 0;
}

uint8_t USBComm::send(uint8_t *buf, uint32_t length)
{
    if (length >= 1)
        return send(buf, &buf[1], length - 1);
    else
        return USBD_FAIL;
}

uint8_t USBComm::send(char *str)
{
    uint8_t cmd = 's';
    uint32_t datalength = strlen(str);
    uint8_t *data = (uint8_t *)(str);
    return send(&cmd, data, datalength);
}

uint8_t USBComm::send(char cmd, uint8_t *data, uint32_t datalength)
{
    return send((uint8_t *)(&cmd), data, datalength);
}

void USBComm::dataAnalysis()
{
    userComm = this;
    if (_isFrameData)
        msgAnalysis(_rxFrame._cmdPtr[0], _rxFrame._dataPtr, _rxFrame._dataLen);
    else
        msgAnalysis(SHELL_CMD, _rxFrame._header, _rxFrame._dataLen);
}
