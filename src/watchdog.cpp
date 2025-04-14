/************************************************************
 * FileName:        watchdog.cpp
 * Description:     看门狗
 *                  STM32 
 * Auther:          Jinsheng
 * CreateDate:      2021-09-05
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/
#include "watchdog.h"

bool watchdogEnabled = false;
/**
 * iwatchdog period = Reload / (LSI/prescaler) --> f(iwatchdog) =  f(LSI) / (reload * prescaler)
 * */
void iwatchdog_init(void)
{
    IWDG_HandleTypeDef iwdgHandle;
    iwdgHandle.Instance = IWDG;
    iwdgHandle.Init.Prescaler = IWDG_PRESCALER_32; // range[4, 256]
    iwdgHandle.Init.Reload = 32000 / 128;          // range [1, 4095]
    HAL_IWDG_Init(&iwdgHandle);
    HAL_IWDG_Refresh(&iwdgHandle);
    watchdogEnabled = true;
}

void wwatchdog_init(void)
{
    WWDG_HandleTypeDef wwdgHandler;
    // ##-2- Configure the WWDG peripheral ######################################
    /* WWDG clock counter = (PCLK1 (42MHz)/4096)/8) = 1281 Hz (~780 us) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64) otherwise a reset will 
     be generated. 
     WWDG Counter value = 127, WWDG timeout = ~780 us * 64 = 49.9 ms 
     看门狗的刷新时间是780us*[(127-80), (127-64)]，否则将会reset
     */
    wwdgHandler.Instance = WWDG;

    wwdgHandler.Init.Prescaler = WWDG_PRESCALER_8;
    wwdgHandler.Init.Window = 80;
    wwdgHandler.Init.Counter = 127;

    HAL_WWDG_Init(&wwdgHandler);
    //HAL_WWDG_Refresh(&wwdgHandler);
}