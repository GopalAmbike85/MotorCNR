
#ifndef ELEC_ANGLE_H
#define ELEC_ANGLE_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Shared state (defined in elec_angle.c)
 * ------------------------------------------------------------------------- */

/* Forced-angle / open-loop hooks, read by the FOC ISR */
extern volatile uint8_t  g_force_angle_en;   /* 1 = force hElAngle to g_force_angle   */
extern volatile int16_t  g_force_angle;      /* forced electrical angle (s16)         */
extern volatile int16_t  g_openloop_id;      /* open-loop d-axis current ref (s16)    */
extern volatile int16_t  g_el_raw_dbg;       /* raw electrical angle, set by FOC ISR  */

/* Race-free Id clear on open-loop exit (27-09-2026): set by open_loop_exit()
 * (elec_angle.c), served and cleared by MC_APP_PostMediumFrequencyHook_M1()
 * (foc_current_control.c), which zeroes FOCVars[M1].Iqdref.d from inside the
 * MF task, after FOC_CalcCurrRef() has written back its copy. */
extern volatile uint8_t  g_id_clear_req;

/* ---------------------------------------------------------------------------
 * Commutation offset auto-calibration (see elec_angle.c)
 * ------------------------------------------------------------------------- */

/** Measures the commutation offset with an open-loop +/-1 el rev sweep,
 *  applies it to the MCSDK encoder (ENC_SetMecAngle) if trustworthy, then
 *  restarts the MCSDK homing. Call from a task as soon as the motor is in RUN.
 *  @retval 0 = applied, else an error code (offset left at 0). */
uint8_t commutation_calibrate(void);

/** Last calibration result: 0xFF not run, 0 applied, 1..4 error codes. */
extern volatile uint8_t g_cal_status;
/** Last measured commutation offset, s16 electrical (for logs/debugger). */
extern volatile int16_t g_cal_offset;
/** 1 = apply the offset to the MCSDK encoder, 0 = measure and print only. */
extern volatile uint8_t g_cal_apply;

#endif /* ELEC_ANGLE_H */
