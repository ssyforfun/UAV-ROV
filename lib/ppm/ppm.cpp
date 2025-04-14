/************************************************************
 * FileName:        PPM.cpp
 * Description:     PPM algorithm
 *                  0.5ms-1.5ms or 1ms-2ms,
 *                  about 20ms per period
 *                  ending high level period more than 2ms
 *                  each less than 2ms high level signal represents a channel speed
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-06-21
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "ppm.h"
#include "userSysTick.h"

#define PPM_MAX_TON (2.1E-3F)
#define PPM_LOW_TON (0.5E-3F)
#define PPM_HIGH_TON (1.5E-3F)
const float PPM_SPEED_GAIN = 2 / (PPM_HIGH_TON - PPM_LOW_TON);
// #define PPM_SPEED_GAIN (2000.0F) // 1/(PPM_HIGH_TON - PPM_LOW_TON)

#ifndef _constrain
#define _constrain(amt, low, high) ((amt) = ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))))
#endif

/**
 * @brief
 * @param
 * @return
 * */
PPM_CLASS::PPM_CLASS()
{
    _isReady = false;
    _isUpdated = false;
    ppmChannels = 0;
}

void PPM_CLASS::init(float pwmstep)
{
    _pwmStep = pwmstep;
    _isReady = true;
}

void PPM_CLASS::pwmCallback(bool isPeriod, uint32_t value)
{
    if (!_isReady)
        return;

    if (!isPeriod)
    {
        float on = _pwmStep * value;
        if (on > PPM_MAX_TON)
        {
            if (_index > 0)
            {
                _receivedTick++;
                ppmChannels = _index;
                _index = 0;
                for (int i = 0; i < PPM_MAX_NUM; i++)
                {
                    speed[i] = (_ton[i] - PPM_LOW_TON) * PPM_SPEED_GAIN - 1.0f;
                }
                _isUpdated = true;
            }
        }
        else if (_index < PPM_MAX_NUM)
        {
            _ton[_index++] = on;
        }
    }
}

bool PPM_CLASS::IsUpdated()
{
    if (_isUpdated)
    {
        _isUpdated = false;
        return true;
    }
    else
        return false;
}

PPM_CLASS ppmInst;
