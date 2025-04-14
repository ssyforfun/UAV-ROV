#ifndef watchdog_h
#define watchdog_h

#include "stm32f4xx_hal.h"

void iwatchdog_init(void);
#define IWDG_Refresh() (IWDG->KR = IWDG_KEY_RELOAD)

extern bool watchdogEnabled;

#endif