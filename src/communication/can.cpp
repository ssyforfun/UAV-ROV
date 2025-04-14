
#include "can.h"
#include "main.h"

#define CAN_BUADRATE (500) // (500) (1000)
CanComm canComm[2];

static void vTaskCanRxProcess(void *pvParameters);
static void vTaskCanComputerSend(void *pvParameters);

// EXTID: 0-7bit代表功能码, 8-12bit:设备索引, 13-28:设备类型
#define COMPUTER_CAN_ID (0x0A5A0000) // 与主机通讯的设备码, 代表通讯方式遵从 串口电脑协议
#define COMPUTER_CAN_ID_MASK (0x0000FFFF << 13)
#define COMPUTER_HOST_CAN_ID_INDEX (0x01 << 8)
#define COMPUTER_DEVICE_CAN_ID_INDEX (0x02 << 8) // 设备索引 1为电脑, 2-31为can设备
#define COMPUTER_CAN_ID_INDEX_MASK (0x00001F00)

// can测试：把当前文件放到setup while()里面
void can_test(uint8_t id)
{
    char info[256];
    for (int i = 0; i < 2; i++)
    {
        if (canComm[i].rxFifo.occupiedSize() > 0) // 发送到电脑的数据会被另外一个CAN2/1捕获, 并存储到rxFifo中
        {
            int len = canComm[i].rxFifo.occupiedSize();
            canComm[i].rxFifo.read((uint8_t *)info, len);
            if (userComm != NULL)
                userComm->rxAnalysis((uint8_t *)info, len);
        }
    }
    osDelay(10);
    // canComm[id].send('z', (uint8_t *)info, 0); // 决定CAN1/2 发送数据到电脑
    info[0] = 'b';
    info[1] = 'c';
    float ftmp[4] = {1.0f, 2.0f, 3.0f, 6.5f};
    float2bytes(ftmp, (uint8_t *)&info[2], 4);
    canComm[id].send('c', (uint8_t *)info, 2 + 4 * 4);
    osDelay(1000);
}

void can_init()
{
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();

    canComm[0].CanHandle.Instance = CAN1;
    canComm[1].CanHandle.Instance = CAN2;

    CAN_HandleTypeDef *canHandle;

    for (int i = 0; i < 2; i++)
    {
        canHandle = &canComm[i].CanHandle;
        CAN_FilterTypeDef sFilterConfig;

        // ##-1- Configure the CAN peripheral #######################################
        canHandle->Init.TimeTriggeredMode = DISABLE;
        canHandle->Init.AutoBusOff = DISABLE;
        canHandle->Init.AutoWakeUp = DISABLE;
        canHandle->Init.AutoRetransmission = ENABLE;    // ENABLE:会一直发送，直到发送成功(ACK位为显性位[=0]); DISABLE:只发送一次，成功TXOK, 无ACK则报错
        canHandle->Init.ReceiveFifoLocked = DISABLE;    // DISABLE:接收溢出后会覆盖先接收的数据
        canHandle->Init.TransmitFifoPriority = DISABLE; // ENABLE:TX邮箱设置成fifo模式, DISABLE:独立邮箱模式
        canHandle->Init.Mode = CAN_MODE_NORMAL;
        // 计算 bit timing
        //-- 1+TimeSeg1+TimeSeg2 = 1bit时间 = N*tq, N>=8, range[8, 25];
        //-- tq = 1/can_clock * prescaler = 1/APB1 * prescaler
        //-- example: 1bit is 1us, 1Mbit/s --> N*tq=1us, 1/tq=42MHz/prescaler, 那么prescaler有3, 6, 7, 14可选, 选3则1/tq=14, N=14
        //-- 采样点在timeSeq1之后
        //-- 采样点补偿范围SyncJumpWidth = [1, 4]
        canHandle->Init.SyncJumpWidth = CAN_SJW_3TQ; // range [1, 4]
        if (CAN_BUADRATE == 1000)
        {
            canHandle->Init.Prescaler = 3; // CAN1/2-->APB1=[168/4=42MHz] --> [for 1Mbit/s-->N*M=42=14*3->N=(1+timeSeg1+timeSeg2)>=8, M=prescaler]
            canHandle->Init.TimeSeg1 = CAN_BS1_9TQ;
            canHandle->Init.TimeSeg2 = CAN_BS2_4TQ;
        }
        else if (CAN_BUADRATE == 500)
        {
            canHandle->Init.Prescaler = 6;
            canHandle->Init.TimeSeg1 = CAN_BS1_9TQ;
            canHandle->Init.TimeSeg2 = CAN_BS2_4TQ;
        }
        HAL_CAN_Init(canHandle);

        if (canHandle->Instance == CAN1) // CAN1才有filter, CAN2是slave CAN, 没有filter
        {
            // ##-2- Configure the CAN Filter ###########################################
            sFilterConfig.FilterBank = 0; // 第几个滤波器组，总共28个。range [0, 27]
            sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
            sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
            // filter 定义
            // -- 每一位代表的意义
            // ---- 32 bit: bit[31:21]->STID[10:0]; bit[20:3]-->EXID[17:0]; bit[2:0]-->[IDE, RTR, 0]
            // ---- 16 bit: bit[15:05]->STID[10:0]; bit[04:03]-->[RTR, IDE]; bit[02:00]-->EXID[17:15]
            //              bit[31:21]->STID[10:0]; bit[20:19]-->[RTR, IDE]; bit[18:16]-->EXID[17:15]
            sFilterConfig.FilterIdHigh = 0x0000;
            sFilterConfig.FilterIdLow = 0x0000;
            sFilterConfig.FilterMaskIdHigh = 0x0000;
            sFilterConfig.FilterMaskIdLow = 0x0000;
            sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
            sFilterConfig.FilterActivation = ENABLE;
            sFilterConfig.SlaveStartFilterBank = 14;
            HAL_CAN_ConfigFilter(canHandle, &sFilterConfig);
            sFilterConfig.FilterBank = 2; // 第几个滤波器组，总共28个。range [0, 27]
            sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
            HAL_CAN_ConfigFilter(canHandle, &sFilterConfig);
        }
        else if (canHandle->Instance == CAN2)
        {
            // ##-2- Configure the CAN Filter ###########################################
            sFilterConfig.FilterBank = 14; // 第几个滤波器组，总共28个。range [0, 27]
            sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
            sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
            // filter 定义
            // -- 每一位代表的意义
            // ---- 32 bit: bit[31:21]->STID[10:0]; bit[20:3]-->EXID[17:0]; bit[2:0]-->[IDE, RTR, 0]
            // ---- 16 bit: bit[15:05]->STID[10:0]; bit[04:03]-->[RTR, IDE]; bit[02:00]-->EXID[17:15]
            //              bit[31:21]->STID[10:0]; bit[20:19]-->[RTR, IDE]; bit[18:16]-->EXID[17:15]
            sFilterConfig.FilterIdHigh = 0x0000;
            sFilterConfig.FilterIdLow = 0x0000;
            sFilterConfig.FilterMaskIdHigh = 0x0000;
            sFilterConfig.FilterMaskIdLow = 0x0000;
            sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
            sFilterConfig.FilterActivation = ENABLE;
            sFilterConfig.SlaveStartFilterBank = 14;
            HAL_CAN_ConfigFilter(canHandle, &sFilterConfig);
            sFilterConfig.FilterBank = 2; // 第几个滤波器组，总共28个。range [0, 27]
            sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
            HAL_CAN_ConfigFilter(canHandle, &sFilterConfig);
        }

        // ##-3- Start the CAN peripheral ###########################################
        HAL_CAN_Start(canHandle);

        // ##-4- Activate CAN RX notification #######################################
        //---- 如果接收的fifo满了，则要马上处理，防止overrun造成数据丢失；其余时刻则是轮询即可
        HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

        // ##-5- Configure Transmission process #####################################*/
        // TxHeader.StdId = 0x321;
        // TxHeader.ExtId = 0x01;
        // TxHeader.RTR = CAN_RTR_DATA;
        // TxHeader.IDE = CAN_ID_STD;
        // TxHeader.DLC = 2;
        // TxHeader.TransmitGlobalTime = DISABLE;
        //  HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox)
        //  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData)
    }

    canComm[0].init();
    canComm[1].init();

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, CAN_Prior, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, CAN_Prior, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, CAN_Prior, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);

    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, CAN_Prior, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}

void can1RxCallback()
{
    CanComm *cancomm = &canComm[0];
    CAN_HandleTypeDef *hcan = &canComm[0].CanHandle;

    // 查看错误
    uint32_t err = hcan->Instance->ESR;
    // ---- 不是自动离线管理时的bus off错误
    if ((hcan->Instance->MCR & CAN_MCR_ABOM) == 0)
    {
        if (err & CAN_MCR_ABOM)
        {
            SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
            uint32_t tickstart = HAL_GetTick();
            while ((hcan->Instance->MSR & CAN_MSR_INAK) == 0U)
            {
                if ((HAL_GetTick() - tickstart) > 10)
                    break;
            }
            CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
        }
    }

    // 读寄存器查看是否有数据
    // uint32_t rf0r;
    while (hcan->Instance->RF0R & 0x03)
    {
        hcan->State = HAL_CAN_STATE_READY;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &cancomm->RxHeader, cancomm->RxData);
        cancomm->rxSignalSet();
    }

    while (hcan->Instance->RF1R & 0x03)
    {
        hcan->State = HAL_CAN_STATE_READY;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &cancomm->RxHeader, cancomm->RxData);
        cancomm->rxSignalSet();
    }
}

void can2RxCallback()
{
    CanComm *cancomm = &canComm[1];
    CAN_HandleTypeDef *hcan = &canComm[1].CanHandle;

    // 查看错误
    uint32_t err = hcan->Instance->ESR;
    // ---- 不是自动离线管理时的bus off错误
    if ((hcan->Instance->MCR & CAN_MCR_ABOM) == 0)
    {
        if (err & CAN_MCR_ABOM)
        {
            SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
            uint32_t tickstart = HAL_GetTick();
            while ((hcan->Instance->MSR & CAN_MSR_INAK) == 0U)
            {
                if ((HAL_GetTick() - tickstart) > 10)
                    break;
            }
            CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
        }
    }

    // 读寄存器查看是否有数据
    // uint32_t rf0r;
    while (hcan->Instance->RF0R & 0x03)
    {
        hcan->State = HAL_CAN_STATE_READY;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &cancomm->RxHeader, cancomm->RxData);
        cancomm->rxSignalSet();
    }

    while (hcan->Instance->RF1R & 0x03)
    {
        hcan->State = HAL_CAN_STATE_READY;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &cancomm->RxHeader, cancomm->RxData);
        cancomm->rxSignalSet();
    }
}

/// 消息处理任务
static void vTaskCanRxProcess(void *pvParameters)
{
    CanComm *hcancomm = (CanComm *)pvParameters;

    while (1)
    {
        if (osSignalWait(1, osWaitForever).status == osEventSignal)
        {
            if ((hcancomm->CanHandle.Instance == CAN1) || (hcancomm->CanHandle.Instance == CAN2))
                hcancomm->rxProcess(); // 如果hcan==null, 会造成不断进入中断，死循环
        }
    }
}

static void vTaskCanComputerSend(void *pvParameters) // send to computer
{
    CanComm *hcancomm = (CanComm *)pvParameters;

    uint8_t sendbuf[8];
    uint32_t computerID = COMPUTER_HOST_CAN_ID_INDEX + COMPUTER_CAN_ID;
    int len;
    int failcnt = 0;
    int failperiod = 10;

    while (1)
    {
        osMutexWait(hcancomm->canComputerTxMutexId, osWaitForever);
        len = hcancomm->sendFifo.occupiedSize();
        // 代表有数据要发送
        if (len >= 1)
        {
            len = (len > 8) ? 8 : len;
            len = hcancomm->sendFifo.read(sendbuf, len);
            failcnt = 0;
            while (hcancomm->sendDataExtID(computerID, sendbuf, len) != 0)
            {
                failcnt++;
                if (failcnt >= failperiod)
                {
                    break;
                }
                osDelay(1); // 间隔一秒重新发送
            }
            osMutexRelease(hcancomm->canComputerTxMutexId);
        }
        else
        {
            osMutexRelease(hcancomm->canComputerTxMutexId);
            vTaskDelayMs(1);
        }
    }

    // 如果任务不是永久性的需要调用 vTaskDelete 删除任务
    // vTaskDelete(NULL);
}

CanComm::CanComm()
{
    CanHandle.Instance = NULL;
    txCount = 0;
    rxCount = 0;
    sendFifo.init(256, sendFifoMemory);
    rxFifo.init(256, rxFifoMemory);
}

void CanComm::init()
{
    osMutexDef(canComputerSend);
    canComputerTxMutexId = osMutexCreate(osMutex(canComputerSend)); // 要在任务之前设置，不然可能未初始化就被使用

    osThreadDef(canRxProcess, vTaskCanRxProcess, osPriority::osPriorityNormal, 0, 256);
    can_rx_thread_id = osThreadCreate(osThread(canRxProcess), this);

    osThreadDef(canTxProcess, vTaskCanComputerSend, osPriority::osPriorityNormal, 0, 256);
    can_tx_thread_id = osThreadCreate(osThread(canTxProcess), this);

    osDelay(2);
}

// return: 0->成功添加到邮箱; 1->失败
int CanComm::sendDataStdID(uint32_t stdId, uint8_t *data, int datalen)
{
    if (CanHandle.Instance == NULL)
        return 1;
    CanHandle.State = HAL_CAN_STATE_READY;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle))
    {
        if (datalen < 0)
            return 1;
        if (datalen > 8)
            return 1;
        uint32_t TxMailbox;
        TxHeader.StdId = stdId;
        TxHeader.ExtId = 0;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = datalen;
        TxHeader.TransmitGlobalTime = DISABLE;

        for (int i = 0; i < datalen; i++)
        {
            TxData[i] = data[i];
        }

        if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) == HAL_OK)
        {
            txCount++;
            return 0;
        }
        else
            return 1;
    }
    else
        return 1;
}

// return: 0->成功添加到邮箱; 1->失败
int CanComm::sendDataExtID(uint32_t extId, uint8_t *data, int datalen)
{
    if (CanHandle.Instance == NULL)
        return 1;
    CanHandle.State = HAL_CAN_STATE_READY;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&CanHandle))
    {
        if (datalen < 0)
            return 1;
        if (datalen > 8)
            return 1;
        uint32_t TxMailbox;
        TxHeader.StdId = 0;
        TxHeader.ExtId = extId;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.DLC = datalen;
        TxHeader.TransmitGlobalTime = DISABLE;

        for (int i = 0; i < datalen; i++)
        {
            TxData[i] = data[i];
        }

        if (HAL_CAN_AddTxMessage(&CanHandle, &TxHeader, TxData, &TxMailbox) == HAL_OK)
        {
            txCount++;
            return 0;
        }
        else
            return 1;
    }
    else
        return 1;
}

void CanComm::rxProcess()
{
    rxCount += RxHeader.DLC;
    rxDataAnalysis();
}

void CanComm::rxDataAnalysis(void)
{
    // uint8_t buf[256];
    // // char info[200];
    // // sprintf(info, "received can, stdId=0x%03X, data len %d, data : []\r\n", RxHeader.StdId, RxHeader.DLC);
    // buf[0] = 'r';
    // buf[1] = ((RxHeader.StdId >> 16) & 0xFF);
    // buf[2] = ((RxHeader.StdId >> 8) & 0xFF);
    // buf[3] = ((RxHeader.StdId >> 0) & 0xFF);
    // buf[4] = RxHeader.DLC;
    // for (uint32_t i = 0; i < RxHeader.DLC; i++)
    // {
    //     buf[5 + i] = RxData[i];
    // }
    // userComm->send('c', buf, 5 + RxHeader.DLC);
    if (RxHeader.IDE == CAN_ID_EXT) // 扩展帧
    {
        if ((RxHeader.ExtId & COMPUTER_CAN_ID_MASK) == COMPUTER_CAN_ID)
        { // 代表上位机电脑数据

            if ((RxHeader.ExtId & COMPUTER_CAN_ID_INDEX_MASK) == COMPUTER_DEVICE_CAN_ID_INDEX)
                rxAnalysis(RxData, RxHeader.DLC);
            else
            {
                // 其它can设备发来的消息
                // userComm->send("received can data\r\n");
                uint32_t len = rxFifo.remainedSize();
                if (len < RxHeader.DLC)
                {
                    rxFifo.pop(RxHeader.DLC - len);
                }
                rxFifo.write(RxData, RxHeader.DLC);
            }
        }
        else
        {
            // extid未知数据
        }
    }
    else // 标准帧
    {
        //
    }
}

uint8_t CanComm::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
{
    osMutexWait(canComputerTxMutexId, osWaitForever);
    while (sendFifo.remainedSize() < (getMiniTxFrameLength() + (int)datalength))
    { // 等待其他任务释放fifo空间
        osMutexRelease(canComputerTxMutexId);
        osDelay(1);
        osMutexWait(canComputerTxMutexId, osWaitForever);
    }

    txConstruct(cmd, data, datalength);
    int length;
    const uint8_t *header = _txFrame.getFrame(&length);
    sendFifo.write(header, length);

    osMutexRelease(canComputerTxMutexId);

    return 0;
}

uint8_t CanComm::send(uint8_t *buf, uint32_t length)
{
    if (length >= 1)
        return send(buf, &buf[1], length - 1);
    else
        return USBD_FAIL;
}

uint8_t CanComm::send(char *str)
{
    uint8_t cmd = 's';
    uint32_t datalength = strlen(str);
    uint8_t *data = (uint8_t *)(str);
    return send(&cmd, data, datalength);
}

uint8_t CanComm::send(char cmd, uint8_t *data, uint32_t datalength)
{
    return send((uint8_t *)(&cmd), data, datalength);
}

void CanComm::dataAnalysis()
{
    // userComm = this;
    msgAnalysis(_rxFrame._cmdPtr[0], _rxFrame._dataPtr, _rxFrame._dataLen);
}
