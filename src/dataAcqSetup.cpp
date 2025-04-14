/************************************************************
 * 如何使用 2023-08-30
 * 1 添加 lib/sigalAcquisition
 *   ---- 这会增加 16k ram开销(默认)
 *   ---- 可以修改 .h文件 NUMBER_OF_SIGNALS 和 SIGNALS_FIFO_LENGTH 来改变信号个数和缓存空间
 * 2 在setup()里添加: SignalAquisition::init((CommunicationProtocol **)&userComm); // 初始化信号采集器
 * 3 在中断函数里添加: SignalAquisition::process(isrTs); // 如果用到触发模式采集的话, 需要添加该函数
 * 4 在本文件里重写: triggerConditionCheck() 和 sourceSelect()
 * **********************************************************/

#include "signalAcquisition.h"
#include "main.h"

#define ACQ_TRIGGER_FREQUENCY (10E-3F)

// 触发信号任务
void vThreadDataAquisition(void *pvParameters)
{
    while (1)
    {
        if (osSignalWait(1, 2).status == osEventSignal)
        {
            SignalAquisition::process(1 / ACQ_TRIGGER_FREQUENCY);
        }
    }
}

// 重写触发信号
// 该成员函数为 静态 类型
// 返回: 0 --> 触发, -1 --> 不触发
int SignalAquisition::triggerConditionCheck(SignalAquisition *sacq)
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

// 重写 数据源 成员函数
int SignalAquisition::sourceSelect(void)
{
    //_sampleData.isPointVal = false; // 默认false
    switch (_sampleData.source)
    {
    case 0: // en = false
    {
        // example
        // _sampleData.pvalue = &uartSensor.temperature; // 姿态角度Y
        // _sampleData.value = (float)servoMotor.Position(); // pos [turn]
        // _sampleData.isPointVal = true;
        break;
    }
    case 1: // imu zitai x
    {
        _sampleData.pvalue = &uartImu.imuHandle.zitai[0];
        //_sampleData.value = uartImu.imuHandle.zitai[0];
        _sampleData.isPointVal = true;
        break;
    }
    case 2: // imu zitai y
    {
        //_sampleData.pvalue = &uartImu.imuHandle.zitai[1];
        _sampleData.value = uartImu.imuHandle.zitai[1];
        _sampleData.isPointVal = false;
        break;
    }
    case 3: // imu zitai z
    {
        //_sampleData.pvalue = &uartImu.imuHandle.zitai[2];
        _sampleData.value = uartImu.imuHandle.zitai[2];
        _sampleData.isPointVal = false;
        break;
    }
    case 4: // TOF longest distance
    {
        _sampleData.pvalue = &uartTof.longestDistance[0];
        //_sampleData.value = uartTof.longestDistance[0];
        _sampleData.isPointVal = true;
        break;
    }
    case 5: // pressure
    {
        _sampleData.pvalue = &pressure;
        //_sampleData.value = pressure;
        _sampleData.isPointVal = true;
        break;
    }
    case 6: // temperature
    {
        _sampleData.pvalue = &temperature;
        //_sampleData.value = temperature;
        _sampleData.isPointVal = true;
        break;
    }
    default:
        break;
    }

    return 0;
}
