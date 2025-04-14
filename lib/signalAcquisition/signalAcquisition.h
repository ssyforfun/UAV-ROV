#ifndef _signalAcquisition_h
#define _signalAcquisition_h

#include <stddef.h>
#include <stdint.h>
#include "protocol.h"
#include "fifo.hpp"
#include "cmsis_os.h"

#define NUMBER_OF_SIGNALS (4)      // 信号数量
#define SIGNALS_FIFO_LENGTH (1024) // 信号的缓存大小

// 用法:
// 0 在signalAcquisiton里设置信号个数 #define SAMPLE_DATA_NUM (4)
// 1 在setup函数里进行串口实例配置 init(userComm);
// 2 在 中断函数里添加 SignalAquisition.Process(isrTs);
// 3 重写 customerSourceSelect()
// 4 上位机发送配置代码, receiveCmd()接收代码, 并配置
// 5 如果需要触发控制, 重写 triggerConditionCheck()
class SignalAquisition
{
public:
    SignalAquisition();

#pragma region staitc part
public:
    static void init(CommunicationProtocol **com);
    static void stop(void);
    static void start(void);
    static void process(float isrTs); // 放置于外部的一个循环(loop)函数里面
    static void process(void);
    static int triggerConditionCheck(SignalAquisition *sacq); //
    static int send(uint8_t *buf, int len);                   // 数据发送
    static void receiveCmd(uint8_t *buf);                     // 命令接收

private:
    static int commandSend(uint8_t *buf, int len); // 命令发送
    static CommunicationProtocol **comm;
    static bool En;
    static osThreadId uploadThreadID;
    static osThreadId innerLoopThreadID;
#pragma endregion static part

#pragma region 通用 part
public:
    int sourceSelect(void);
    int sourceSelect(int sourceindex);

public:
    struct DataConstruct
    {
        float period;      // 采样周期
        float tick;        // 采样计时
        Fifo<float> fifo;  // 缓存处理
        float value;       // 值1
        float *pvalue;     // 值2
        bool isPointVal;   // [false: 数据从value走, 需要用sourceSelect来选择], [true: 数据从pvalue走, 效率更高, 但局限用于float类型]
        int source;        // 数据来源索引
        int mode;          // [0: normal mode]; [1: trigger mode]
        int triggerState;  // 触发状态 [0:信号采集] [1:触发完成]
        int triggerLength; // 触发采集的长度
        int triggerCnt;    // 触发采集计数
        int triggerSignal; // 触发信号源
        float zitaiX;
        float zitaiY;
        float zitaiZ;
    };
    DataConstruct _sampleData;

private:
    float _memory[SIGNALS_FIFO_LENGTH];
#pragma endregion 通用 part
};

extern SignalAquisition signalAcq[NUMBER_OF_SIGNALS];

#endif