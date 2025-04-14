#ifndef pid_h
#define pid_h

#include <stm32f4xx_hal.h>

class PID_CLASS
{
public:
    PID_CLASS();

public:
    float ref, fb, err, lastErr, last2Err;
    float kp, ki, kd;
    float vc, vcp, vci, vcd;
    float vcmax, vcmin;

    float unifyK;
    bool isUnified;
    bool isManual;
    bool isIncrement;

    bool decayEn;
    float decayFactor;
public:
    void init(void);
    float calc(float fb);
    float calcWithAW(float fb, float ts);
    float manualOut(float input);
    void clearErr(void);
};

#endif