#ifndef APP_TIME_H
#define APP_TIME_H

/* ---------------------------------------------------------------------------
 * Tick-rate consistency check for this project (26-09-2026).
 *
 * The MCSDK overrides vPortSetupTimerInterrupt() (Src/motorcontrol.c) so the
 * FreeRTOS SysTick fires at SYS_TICK_FREQUENCY = 2000 Hz (every 500 us).
 * FreeRTOSConfig.h keeps configTICK_RATE_HZ = 1000 (CMSIS-RTOS v1 needs
 * portTICK_PERIOD_MS >= 1) and instead overrides the standard pdMS_TO_TICKS()
 * with APP_RTOS_TICK_HZ, so pdMS_TO_TICKS(x) now gives REAL milliseconds.
 * (Before this, application code used a private APP_MS_TO_TICKS() macro.)
 *
 * This header only guards that the two tick rates match. Include it in any
 * application file that uses pdMS_TO_TICKS().
 *
 * NOTE: CMSIS-RTOS v1 osDelay(ms) does NOT use pdMS_TO_TICKS(): it divides by
 * portTICK_PERIOD_MS (= 1), so osDelay(x) is still x ticks = x/2 real ms.
 * Use vTaskDelay(pdMS_TO_TICKS(x)) in application code.
 * ------------------------------------------------------------------------- */
#include <stdint.h>
#include "FreeRTOS.h"
#include "parameters_conversion.h"   /* SYS_TICK_FREQUENCY */

_Static_assert(APP_RTOS_TICK_HZ == SYS_TICK_FREQUENCY,
               "APP_RTOS_TICK_HZ (FreeRTOSConfig.h) must equal SYS_TICK_FREQUENCY (parameters_conversion.h)");

/* Self-test of the override (compile time): 1000 ms must be 2000 ticks. */
_Static_assert(pdMS_TO_TICKS(1000U) == (TickType_t)APP_RTOS_TICK_HZ,
               "pdMS_TO_TICKS() is not the application override (FreeRTOSConfig.h USER CODE Defines)");

#endif /* APP_TIME_H */
