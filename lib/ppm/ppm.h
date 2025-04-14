/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-09-20 10:55:01
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-20 14:33:56
 * @FilePath: \demo3\lib\ppm\ppm.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef ppm_h
#define ppm_h

#include <stm32f4xx_hal.h>

#define PPM_MAX_NUM (20)

class PPM_CLASS
{
public:
    PPM_CLASS();

public:
    void init(float pwmstep);
    void pwmCallback(bool isPeriod, uint32_t value);

public:
    float speed[PPM_MAX_NUM];
    int ppmChannels;
    // 只能读一次, 读后自动置0;
    bool IsUpdated(); 

private:
    float _ton[PPM_MAX_NUM];
    int _index;
    float _pwmStep;
    bool _isUpdated;
    uint64_t _receivedTick;

private:
    bool _isReady;
};

extern PPM_CLASS ppmInst;

#endif