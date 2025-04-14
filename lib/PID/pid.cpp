/************************************************************
 * FileName:        PID.cpp
 * Description:     PID algorithm
 *                  STM32 
 * Auther:          Jinsheng
 * CreateDate:      2021-06-21
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "pid.h"

#ifndef _constrain
#define _constrain(amt, low, high) ((amt) = ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))))
#endif

/**
 * @brief 
 * @param 
 * @return 
 * */
PID_CLASS::PID_CLASS()
{
    init();
}

void PID_CLASS::init()
{
    decayEn = false;
    decayFactor = 0.999f;

    ref = 1.0f;
    fb = 0;
    err = 0;
    lastErr = 0;

    kp = 0;
    ki = 0;
    kd = 0;

    vc = 0;
    vcp = 0;
    vci = 0;
    vcd = 0;

    vcmax = 0;
    vcmin = 0;

    unifyK = 1.0f;
    isUnified = false;
    isManual = false;
    isIncrement = false;
}

void PID_CLASS::clearErr(void)
{
    err = 0;
    lastErr = 0;
    last2Err = 0;
}

float PID_CLASS::manualOut(float input)
{
    vc = input;
    _constrain(vc, vcmin, vcmax);
    clearErr();
    return vc;
}

float PID_CLASS::calc(float feedback)
{
    if (isUnified) // 判断是否是 归一化模式
    {
        fb = feedback * unifyK;
        err = 1.0f - fb;
    }
    else
    {
        fb = feedback;
        err = ref - fb;
    }

    if (isIncrement) // 判断是否是 增量模式
    {
        float perr = err - lastErr;
        float ierr = err;
        float derr = err - 2 * lastErr + last2Err;

        float increment = kp * perr + ki * ierr + kd * derr;
        vc += increment;
        _constrain(vc, vcmin, vcmax);
    }
    else
    {
        vcp = kp * err;
        vci += ki * err;
        vcd = kd * (err - lastErr);
        vc = vcp + vci + vcd;
        if (((vc > vcmax) || (vc < vcmin)) && (decayEn))
        { // 退饱机制
            vci -= ki * err;
            vci *= decayFactor;
        }
        _constrain(vci, vcmin, vcmax);
        _constrain(vc, vcmin, vcmax);
    }

    last2Err = lastErr;
    lastErr = err;

    return vc;
}

/// @brief 带有anti-windup功能的pid计算
/// @param fb 
/// @return 
float PID_CLASS::calcWithAW(float feedback, float ts)
{
    if (isUnified) // 判断是否是 归一化模式
    {
        fb = feedback * unifyK;
        err = 1.0f - fb;
    }
    else
    {
        fb = feedback;
        err = ref - fb;
    }

    if (isIncrement) // 判断是否是 增量模式
    {
        float perr = err - lastErr;
        float ierr = err;
        float derr = err - 2 * lastErr + last2Err;

        float increment = kp * perr + ki * ierr * ts + kd * derr / ts;
        vc += increment;
        _constrain(vc, vcmin, vcmax);
    }
    else
    {
        vcp = kp * err;
        vci += ki * err * ts;
        // vcd = kd * (err - lastErr);  // 不用微分部分
        //vc = vcp + vci + vcd;
        vc = vcp + vci;
        float vcs = vc; // 饱和输出
        _constrain(vcs, vcmin, vcmax);
        float se = vc - vcs; 
        vci -= se; // 让 vci+vcp 始终保持不饱和
        _constrain(vci, vcmin, vcmax);
        vc = vcs;
    }

    last2Err = lastErr;
    lastErr = err;

    return vc;
}
