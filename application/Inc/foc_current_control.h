#ifndef FOC_CURRENT_CONTROL_H
#define FOC_CURRENT_CONTROL_H

#include <stdint.h>

uint16_t APP_FOC_CurrControllerM1(void);

void FOC_LogStatus(const char *tag);

void FOC_GuardReport(void);

extern volatile uint8_t g_i_sense_invert;

extern volatile uint8_t g_i_phase_remap;

extern volatile uint8_t g_i2t_throttled;

#endif
