
#include "signalAcquisition.h"
#include "stdio.h"
#include "string.h"

#define INNERLOOP_DATA_ACQUISITION_INTERVAL (1)      // 1ms 采集周期
#define UPLOAD_DATA_ACQUISITION_INTERVAL (50)        // 50ms 发送一次
#define FREERTOS_PERIOD_ACQUISITION_INTERVAL (1E-3F) // 操作系统的周期

SignalAquisition signalAcq[NUMBER_OF_SIGNALS];

// 上传数据给上位机的任务, 要和 vThreadInnerLoopOfDataAquisition 采集的最快速度匹配 osDealy的时间
void vThreadUploadOfDataAquisition(void *pvParameters)
{
    uint8_t buf[256];
    const int maxLength = 60;
    float fbuf[maxLength];
    int len, sendlen;
    SignalAquisition::DataConstruct *data;
    while (1)
    {
        for (int i = 0; i < NUMBER_OF_SIGNALS; i++)
        {
            data = &signalAcq[i]._sampleData;
            len = data->fifo.occupiedSize();
            while (len > 0)
            {
                sendlen = (len < maxLength) ? len : maxLength; // float size
                len -= sendlen;
                if (sendlen > 0)
                {
                    buf[0] = i;
                    buf[1] = sendlen * 4;
                    int actuallen = data->fifo.read(fbuf, sendlen);
                    if (actuallen > 0)
                    {
                        float2bytes(fbuf, &buf[2], actuallen);
                        SignalAquisition::send(buf, actuallen * 4 + 2);
                    }
                }
            }
        }

        osDelay(UPLOAD_DATA_ACQUISITION_INTERVAL);
    }
}

// 该定时器用来做非 trigger 信号的采集
void vThreadInnerLoopOfDataAquisition(void *pvParameters)
{
    while (1)
    {
        SignalAquisition::process();
        osDelay(INNERLOOP_DATA_ACQUISITION_INTERVAL);
    }
}

#pragma region static part
CommunicationProtocol **SignalAquisition::comm = NULL;
bool SignalAquisition::En = false;
osThreadId SignalAquisition::uploadThreadID = NULL;
osThreadId SignalAquisition::innerLoopThreadID = NULL;

void SignalAquisition::init(CommunicationProtocol **com)
{
    comm = com;

    osThreadDef(uploadOfDataAcquisition, vThreadUploadOfDataAquisition, osPriority::osPriorityLow, 0, 512);
    uploadThreadID = osThreadCreate(osThread(uploadOfDataAcquisition), NULL);

    osThreadDef(innerLoopOfDataAcquisition, vThreadInnerLoopOfDataAquisition, osPriority::osPriorityLow, 0, 256);
    innerLoopThreadID = osThreadCreate(osThread(innerLoopOfDataAcquisition), NULL);
}

void SignalAquisition::stop(void)
{
    En = false;
}

void SignalAquisition::start(void)
{
    En = true;
    for (int i = 0; i < NUMBER_OF_SIGNALS; i++)
    {
        signalAcq[i]._sampleData.fifo.reset();
        signalAcq[i]._sampleData.tick = 0;         // 复位
        signalAcq[i]._sampleData.triggerCnt = 0;
        signalAcq[i]._sampleData.triggerState = 0; // 重新使能触发
    }
}

// 触发信号源选择
__attribute__((weak)) int SignalAquisition::triggerConditionCheck(SignalAquisition *sacq)
{
    switch (sacq->_sampleData.triggerSignal)
    {
    case 0: // SignalAcquisition::En
        if (!En)
            return -1;
        break;
    default:
        return -1;
    }

    return 0;
}

void SignalAquisition::process(float isrTs)
{
    if (!En) // 总开关
        return;

    DataConstruct *data;
    for (int i = 0; i < NUMBER_OF_SIGNALS; i++)
    {
        data = &signalAcq[i]._sampleData;

        if (data->source <= 0) // 信号源是否为空
            continue;
        if (data->mode != 1) // 非触发模式
            continue;

        if (data->triggerState > 0) // 触发工作是否完成
            continue;

        if (triggerConditionCheck(&signalAcq[i]) < 0) // 触发信号源 触发判断
            continue;
        
        data->tick += isrTs;
        if (data->tick >= data->period) // 采集周期计算
        {
            data->tick -= data->period;

            if (data->fifo.remainedSize() > 0)
            {
                if (data->isPointVal)
                    data->fifo.write(*data->pvalue);
                else
                {
                    signalAcq[i].sourceSelect();
                    data->fifo.write(data->value);
                }

                if (++data->triggerCnt >= data->triggerLength)
                    data->triggerState = 1;
            }
        }
    }
}

void SignalAquisition::process(void)
{
    if (!En) // 总开关
        return;

    DataConstruct *data;
    for (int i = 0; i < NUMBER_OF_SIGNALS; i++)
    {
        data = &signalAcq[i]._sampleData;

        if (data->source <= 0)
            continue;
        if (data->mode != 0) // 非正常模式
            continue;

        data->tick += INNERLOOP_DATA_ACQUISITION_INTERVAL * FREERTOS_PERIOD_ACQUISITION_INTERVAL;
        if (data->tick >= data->period) // 采集周期计算
        {
            data->tick -= data->period;
            if (data->fifo.remainedSize() > 0)
            {
                if (data->isPointVal)
                    data->fifo.write(*data->pvalue);
                else
                {
                    signalAcq[i].sourceSelect();
                    data->fifo.write(data->value);
                }
            }
        }
    }
}

void SignalAquisition::receiveCmd(uint8_t *buf)
{
    float ftmp[2];
    char command[256];
    char *sbuf = &command[2];
    int numbers, i;
    uint8_t *pbuf;
    DataConstruct *data;

    switch (buf[0])
    {
    case 'a': // all, {numbers, [mode, source, period, triggerLength, triggerSignal], [...], [...]}
    {
        if (buf[1] == 's')
        {
            numbers = (buf[2] < NUMBER_OF_SIGNALS) ? buf[2] : NUMBER_OF_SIGNALS;
            pbuf = &buf[3];
            for (i = 0; i < numbers; i++)
            {
                data = &signalAcq[i]._sampleData;
                data->mode = *pbuf++;
                signalAcq[i].sourceSelect(*pbuf++);
                bytes2float(pbuf, ftmp, 2);
                pbuf += 8;
                data->period = ftmp[0];
                data->triggerLength = (int)(ftmp[1] + 0.1f);
                data->triggerSignal = *pbuf++;
            }

            command[0] = 'A';
            command[1] = 's';
            sprintf(sbuf, "signal(%d) set done\r\n", numbers);
            commandSend((uint8_t *)command, strlen(command));
        }
        else if (buf[1] == 'r')
        {
            command[0] = 'A';
            command[1] = 'r';
            pbuf = (uint8_t *)&command[2];
            *pbuf++ = NUMBER_OF_SIGNALS;
            for (int i = 0; i < NUMBER_OF_SIGNALS; i++)
            {
                data = &signalAcq[i]._sampleData;
                *pbuf++ = data->mode;
                *pbuf++ = data->source;
                ftmp[0] = data->period;
                ftmp[1] = data->triggerLength;
                float2bytes(ftmp, pbuf, 2);
                pbuf += 8;
                *pbuf++ = data->triggerSignal;
            }
            commandSend((uint8_t *)command, NUMBER_OF_SIGNALS * 11 + 3);
        }
        break;
    }
    case 'e': // 总开关
    {
        if (buf[1] == 's')
        {
            if (buf[2] == '1')
            {
                start();
            }
            else if (buf[2] == '0')
            {
                stop();
            }
            command[0] = 'e';
            command[1] = 's';
            if (En)
                sprintf(sbuf, "dataAcq start...\r\n");
            else
                sprintf(sbuf, "dataAcq stop\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        else if (buf[1] == 'r')
        {
            command[0] = 'e';
            command[1] = 'r';
            if (En)
                sprintf(sbuf, "dataAcq start...\r\n");
            else
                sprintf(sbuf, "dataAcq stop\r\n");
            commandSend((uint8_t *)command, strlen(command));
        }
        break;
    }
    default:
        break;
    }
}

int SignalAquisition::send(uint8_t *buf, int len)
{
    if (comm != NULL)
        return (*comm)->send('f', buf, len);
    else
        return -1;
}

int SignalAquisition::commandSend(uint8_t *buf, int len)
{
    if (comm != NULL)
        return (*comm)->send('F', buf, len);
    else
        return -1;
}
#pragma endregion static part

SignalAquisition::SignalAquisition(void)
{
    _sampleData.fifo.init(SIGNALS_FIFO_LENGTH, _memory);

    _sampleData.isPointVal = false;
    _sampleData.period = 0.01f;
    _sampleData.tick = 0;
    _sampleData.value = 0;
    _sampleData.pvalue = &_sampleData.value;
    _sampleData.isPointVal = false;
    _sampleData.source = 0;
    _sampleData.mode = 0;
    _sampleData.triggerState = 0;
    _sampleData.triggerLength = 1;
    _sampleData.triggerCnt = 0;
    _sampleData.triggerSignal = 0;
}

int SignalAquisition::sourceSelect(int sourceindex)
{
    _sampleData.source = sourceindex;
    sourceSelect();
    return 0;
}

// 数据源选择
// 这个是用户自定义的数据源选择
// 更新这个函数方法:
// 0 case 0 是信号开关, 信号要从case 1开始
// 1 把该函数注释掉，然后在其它文件里重新写这个函数，注意其它文件需要引用 #include "signalAcquisition.h"
__attribute__((weak)) int SignalAquisition::sourceSelect(void)
{
    switch (_sampleData.source)
    {
    case 0: // 0 代表无
        break;
    // an example:
    // case 0:
    // {
    //     _sampleData.value = vout;
    //     _sampleData.isPointVal = false;
    //     break;
    // }
    // case 1:
    // {
    //     _sampleData.pvalue = &vout;
    //     _sampleData.isPointVal = true;
    // }
    default:
        _sampleData.source = 0;
        _sampleData.isPointVal = false;
        break;
    }

    return 0;
}
