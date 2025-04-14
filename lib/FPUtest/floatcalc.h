#ifndef floatcalc_h
#define floatcalc_h

#include "stm32f4xx_hal.h"

class FloatCalc
{
public:
    FloatCalc();

private:
    float a, b, c;
    uint32_t multi_times;
    uint32_t div_times;
    uint32_t sin_times;

public:
    float multiMIPS(void);
    float divMIPS(void);
    float sinMIPS(void);
    float sqrtMIPS(void);
    float armsinMIPS(void);
    float armsqrtMIPS(void);
};

#endif
