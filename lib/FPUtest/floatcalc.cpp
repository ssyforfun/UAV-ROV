#include <floatcalc.h>
#include "arm_math.h"
#include "arm_const_structs.h"

FloatCalc::FloatCalc()
{
    a = 1.24;
    b = 34.456;
    c = 0;
    multi_times = 12000000;
    div_times = 500000;
    sin_times = 200000;
}

/**
 * retval: 每秒的指令数
*/
float FloatCalc::multiMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < multi_times; i++)
    {
        c = a * b;
        b = c * a;
        a = b * c;
        c = a * b;
        b = c * a;
        a = b * c;
        c = a * b;
        b = c * a;
        a = b * c;
        c = a * b;
        b = c * a;
        a = b * c;
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    a = (float)tdiff * 0.1f + 0.1f;

    float pMIPS;
    pMIPS = (12 * multi_times) / ((float)tdiff / 1000);

    return pMIPS;
}

float FloatCalc::divMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < div_times; i++)
    {
        c = a / b;
        b = c / a;
        a = b / c;
        c = a / b;
        b = c / a;
        a = b / c;
        c = a / b;
        b = c / a;
        a = b / c;
        c = a / b;
        b = c / a;
        a = b / c;
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    float pMIPS;
    pMIPS = (12 * div_times) / ((float)tdiff / 1000);

    a = (float)tdiff * 0.1f + 0.1f;
    b = 0.2f;

    return pMIPS;
}

float FloatCalc::sinMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < sin_times; i++)
    {
        c = sinf(a);
        b = sinf(b);
        a = sinf(c);
        c = sinf(a);
        b = sinf(c);
        a = sinf(b);
        c = sinf(a);
        b = sinf(c);
        a = sinf(b);
        c = sinf(a);
        b = sinf(c);
        a = sinf(b);
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    float pMIPS;
    pMIPS = (12 * sin_times) / ((float)tdiff / 1000);

    a = (float)tdiff * 0.1f + 0.1f;
    b = 0.2f + a;

    return pMIPS;
}

float FloatCalc::armsinMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < sin_times; i++)
    {
        c = arm_sin_f32(a);
        b = arm_sin_f32(b);
        a = arm_sin_f32(c);
        c = arm_sin_f32(a);
        b = arm_sin_f32(c);
        a = arm_sin_f32(b);
        c = arm_sin_f32(a);
        b = arm_sin_f32(c);
        a = arm_sin_f32(b);
        c = arm_sin_f32(a);
        b = arm_sin_f32(c);
        a = arm_sin_f32(b);
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    float pMIPS;
    pMIPS = (12 * sin_times) / ((float)tdiff / 1000);

    a = (float)tdiff * 0.1f + 0.1f;
    b = 0.2f + a;

    return pMIPS;
}

float FloatCalc::armsqrtMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < div_times; i++)
    {
        arm_sqrt_f32(a, &c);
        arm_sqrt_f32(b, &a);
        arm_sqrt_f32(c, &b);
        arm_sqrt_f32(a, &c);
        arm_sqrt_f32(b, &a);
        arm_sqrt_f32(c, &b);
        arm_sqrt_f32(a, &c);
        arm_sqrt_f32(b, &a);
        arm_sqrt_f32(c, &b);
        arm_sqrt_f32(a, &c);
        arm_sqrt_f32(b, &a);
        arm_sqrt_f32(c, &b);
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    float pMIPS;
    pMIPS = (12 * div_times) / ((float)tdiff / 1000);

    c = (float)tdiff * 0.1f + 0.1f;
    a = 0.2f;

    return pMIPS;
}

float FloatCalc::sqrtMIPS(void)
{
    uint32_t tstart, tend, tdiff, i;

    tstart = HAL_GetTick();
    for (i = 0; i < div_times; i++)
    {
        c = sqrtf(a);
        b = sqrtf(b);
        a = sqrtf(c);
        c = sqrtf(a);
        b = sqrtf(b);
        a = sqrtf(c);
        c = sqrtf(a);
        b = sqrtf(b);
        a = sqrtf(c);
        c = sqrtf(a);
        b = sqrtf(b);
        a = sqrtf(c);
    }
    tend = HAL_GetTick();

    tdiff = tend - tstart;

    float pMIPS;
    pMIPS = (12 * div_times) / ((float)tdiff / 1000);

    a = (float)tdiff * 0.1f + 0.1f;
    b = 0.2f;

    return pMIPS;
}