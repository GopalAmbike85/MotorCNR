#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "parameters_conversion.h"

_Static_assert(APP_RTOS_TICK_HZ == SYS_TICK_FREQUENCY,
               "APP_RTOS_TICK_HZ (FreeRTOSConfig.h) must equal SYS_TICK_FREQUENCY (parameters_conversion.h)");

_Static_assert(pdMS_TO_TICKS(1000U) == (TickType_t)APP_RTOS_TICK_HZ,
               "pdMS_TO_TICKS() is not the application override (FreeRTOSConfig.h USER CODE Defines)");

#endif
