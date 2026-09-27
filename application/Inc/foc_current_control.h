#ifndef FOC_CURRENT_CONTROL_H
#define FOC_CURRENT_CONTROL_H

#include <stdint.h>

/* Application FOC current controller, called from the strong
 * FOC_HighFrequencyTask() override in foc_current_control.c. */
uint16_t APP_FOC_CurrControllerM1(void);

/* Prints one line of drive status (position, Iq/Id, encoder, index count).
 * Call from a task, e.g. every 100 ms while homing. */
void FOC_LogStatus(const char *tag);

/* Prints the runaway-guard trip reason once (no-op otherwise). Call from a task loop. */
void FOC_GuardReport(void);

/* 1 = measured phase currents are negated (set by the self-test) */
extern volatile uint8_t g_i_sense_invert;

/* 1 = re-order measured phase currents (diagnostic, see foc_current_control.c) */
extern volatile uint8_t g_i_phase_remap;

/* 1 while the i2t limiter holds Iq at nominal */
extern volatile uint8_t g_i2t_throttled;

#endif /* FOC_CURRENT_CONTROL_H */