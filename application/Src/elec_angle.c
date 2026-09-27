
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "mc_type.h"

#include "elec_angle.h"
#include "app_time.h"   /* pdMS_TO_TICKS(): real ms (tick is 2 kHz) */

/* ---------------------------------------------------------------------------
 * Shared state (declared in elec_angle.h)
 * ------------------------------------------------------------------------- */
volatile uint8_t g_force_angle_en = 0;   /* 1 = force hElAngle to g_force_angle */
volatile int16_t g_force_angle    = 0;   /* forced electrical angle (s16) when enabled */
volatile int16_t g_el_raw_dbg     = 0;   /* raw pre-offset electrical angle, for debug */
volatile int16_t g_openloop_id    = 0;   /* open-loop d-axis current ref (s16); 0 = off */
volatile uint8_t g_id_clear_req   = 0U;  /* 1 = MF hook must zero Iqdref.d (race fix) */

/* Max wait for the MF hook to confirm the Id clear. The MF task runs every
 * 1 ms, so 20 ms is ample even with some starvation of its idle priority. */
#define OL_EXIT_CONFIRM_MS   20U

/**
 * @brief  Leaves the open-loop forced-angle mode and removes the open-loop
 *         d-axis current.
 * @note   Fix for the "Id stays on after the search" bug: the FOC ISR writes
 *         FOCVars[M1].Iqdref.d only while g_force_angle_en is set, and
 *         FOC_CalcCurrRef() rewrites only .q, so the last open-loop Id value
 *         was left in force indefinitely. It is cleared here explicitly.
 *         Interrupts are masked so the FOC ISR cannot run between clearing
 *         the enable flag and clearing the reference.
 * @note   Race fix (27-09-2026): FOC_CalcCurrRef() (MF task, IDLE priority)
 *         copies the whole Iqdref, computes, then writes the copy back in two
 *         separate critical sections. This task (higher priority) can pre-empt
 *         it in between; the write-back then restores the stale Id = CAL_ID_A
 *         and nothing clears it again. So the clear is ALSO requested from the
 *         MF task itself (g_id_clear_req, served by
 *         MC_APP_PostMediumFrequencyHook_M1() in foc_current_control.c, which
 *         runs right after FOC_CalcCurrRef() in the same task, so no stale
 *         copy can outlive it). This waits for that confirmation.
 * @retval 1 if the MF task confirmed the clear, 0 on timeout (the request
 *         stays pending and is still served on a later MF tick).
 */
static uint8_t open_loop_exit(void)
{
  uint32_t t0;

  __disable_irq();
  g_force_angle_en        = 0U;
  g_openloop_id           = 0;
  FOCVars[M1].Iqdref.d    = 0;
  g_id_clear_req          = 1U;      /* MF hook clears .d again, race-free */
  __enable_irq();

  t0 = HAL_GetTick();
  while ((0U != g_id_clear_req) && ((HAL_GetTick() - t0) < OL_EXIT_CONFIRM_MS))
  {
    vTaskDelay(pdMS_TO_TICKS(1));    /* let the MF task run */
  }

  if (0U != g_id_clear_req)
  {
    printf("[OL] WARNING: Id clear not confirmed by MF task within %u ms (still pending)\r\n",
           (unsigned)OL_EXIT_CONFIRM_MS);
    return 0U;
  }
  return 1U;
}

/**
 * @brief  Waits ms (10 ms steps) while the motor stays in RUN.
 * @retval 1 if still in RUN, 0 if a fault/guard stopped the motor.
 */
static uint8_t diag_wait_ms(uint32_t ms)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < ms)
  {
    if (RUN != MCI_GetSTMState(pMCI[M1])) { return 0U; }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return (RUN == MCI_GetSTMState(pMCI[M1])) ? 1U : 0U;
}

/* ===========================================================================
 * Commutation offset auto-calibration (26-09-2026)
 * ===========================================================================
 * WHY: bench runs 7 and 8 showed the rotor settling 21-23 deg and ~37 deg
 * AHEAD of the field after the MCSDK alignment, with the same sign in both
 * sweep directions (so a fixed offset, not friction) and a different value
 * each boot (the alignment does not land in the same place every time).
 * That costs torque (cos 37 deg = 0.80) and adds d/q cross-coupling.
 *
 * HOW (commutation_calibrate(), called once as soon as the motor is in RUN):
 *   1. Pause the MCSDK Z search that started on entry to RUN, and set
 *      AlignmentStatus = TC_AWAITING_FOR_ALIGNMENT so index pulses during
 *      the sweep do NOT set position zero.
 *   2. Open loop at CAL_ID_A: lock the rotor to the field, then turn the
 *      field +1 electrical rev and -1 back (2 s each). At every step after
 *      the first CAL_SKIP_STEPS, record (field angle - raw encoder angle).
 *      Sweeping both ways cancels friction lag; a full rev averages cogging.
 *   3. Accept only if the encoder followed (>= CAL_MIN_COUNTS each way, in
 *      the field's direction) and the scatter is within CAL_MAX_SPREAD.
 *      Then TELL THE MCSDK ENCODER the corrected angle with ENC_SetMecAngle()
 *      (offset = mean(field - encoder)), so the MCSDK's own electrical angle
 *      is right and no correction is needed in our FOC code. Otherwise leave
 *      the encoder untouched (behaviour as before).
 *   4. Restart the standard MCSDK homing (TC_EncAlignmentCommand()), now
 *      with corrected commutation. main.c waits for its result as before.
 * The runaway guard (speed check) stays active throughout.
 * ======================================================================== */
#define CAL_ID_A          0.30f     /* A, open-loop d current (< nominal)      */
#define CAL_STEPS         100       /* steps per sweep                         */
#define CAL_STEP          655       /* s16 per step: 100 x 655 ~ 1 el rev      */
#define CAL_STEP_MS       20U       /* ms per step: 2 s per sweep              */
#define CAL_LATCH_MS      300U      /* initial lock-on time                    */
#define CAL_SKIP_STEPS    10        /* first steps of each sweep not recorded
                                     * (rotor still catching up)              */
#define CAL_MIN_COUNTS    900       /* encoder counts per el rev (1024 ideal)  */
#define CAL_MAX_SPREAD    5461      /* s16 = 30 deg: max half-range of samples */
#define CAL_ENC_MODULO    1024      /* TIM4 period = M1_PULSE_NBR (1023) + 1   */

#define CAL_OK            0U
#define CAL_ERR_NOT_RUN   1U
#define CAL_ERR_ABORTED   2U        /* fault / guard during the sweep          */
#define CAL_ERR_FOLLOW    3U        /* encoder did not follow the field        */
#define CAL_ERR_SPREAD    4U        /* samples too scattered                   */

volatile uint8_t g_cal_status = 0xFFU;   /* 0xFF = not run, else CAL_xxx       */
volatile int16_t g_cal_offset = 0;       /* last measured offset (s16, el.)    */
/* 1 = apply a trustworthy offset to the MCSDK encoder; 0 = MEASURE ONLY
 * (print it, change nothing). Set by main.c (APP_COMM_CAL_APPLY) before
 * calling commutation_calibrate(). Added 26-09-2026 to test whether bad
 * boots coincide with large alignment offsets. */
volatile uint8_t g_cal_apply  = 1U;

/** Circular accumulator for s16 angles (valid while spread < 180 deg). */
typedef struct
{
  int16_t  ref;      /* first sample: everything is summed relative to it */
  int32_t  sum;
  int16_t  dmin;
  int16_t  dmax;
  uint16_t n;
} cal_acc_t;

/** @brief Adds one s16 angle sample (int16 wrap keeps +/-180 deg samples together). */
static void cal_acc_add(cal_acc_t *a, int16_t x)
{
  int16_t d;

  if (0U == a->n)
  {
    a->ref  = x;
    a->sum  = 0;
    a->dmin = 0;
    a->dmax = 0;
  }
  d = (int16_t)(x - a->ref);
  a->sum += d;
  if (d < a->dmin) { a->dmin = d; }
  if (d > a->dmax) { a->dmax = d; }
  a->n++;
}

/** @brief Circular mean of the samples added so far (n must be > 0). */
static int16_t cal_acc_mean(const cal_acc_t *a)
{
  return (int16_t)(a->ref + (int16_t)(a->sum / (int32_t)a->n));
}

/** @brief Half of the sample range, s16 (a scatter measure). */
static int16_t cal_acc_spread(const cal_acc_t *a)
{
  return (int16_t)(((int32_t)a->dmax - (int32_t)a->dmin) / 2);
}

/**
 * @brief  Pure evaluation of a calibration (no hardware; unit-testable).
 * @param  fwd, back     accumulators for the + and - sweeps.
 * @param  cnt_fwd/back  encoder counts moved by each sweep.
 * @param  offset_out    out: offset to apply (s16), valid when CAL_OK.
 * @retval CAL_OK, CAL_ERR_FOLLOW or CAL_ERR_SPREAD.
 */
static uint8_t cal_evaluate(const cal_acc_t *fwd, const cal_acc_t *back,
                            int32_t cnt_fwd, int32_t cnt_back, int16_t *offset_out)
{
  int16_t m_f;
  int16_t m_b;

  if ((0U == fwd->n) || (0U == back->n) ||
      (cnt_fwd < CAL_MIN_COUNTS) || (cnt_back > -CAL_MIN_COUNTS))
  {
    return CAL_ERR_FOLLOW;            /* rotor did not follow, or wrong direction */
  }
  if ((cal_acc_spread(fwd) > CAL_MAX_SPREAD) || (cal_acc_spread(back) > CAL_MAX_SPREAD))
  {
    return CAL_ERR_SPREAD;
  }

  /* Midpoint of the two sweep means: friction lag has opposite signs in the
   * two directions and cancels. int16 wrap handles means near +/-180 deg. */
  m_f = cal_acc_mean(fwd);
  m_b = cal_acc_mean(back);
  *offset_out = (int16_t)(m_f + (int16_t)((int16_t)(m_b - m_f) / 2));
  return CAL_OK;
}

/** @brief TIM4 count change since *prev, unwrapped at 1024 (small moves only). */
static int16_t cal_enc_delta(int16_t *prev)
{
  int16_t cnt = (int16_t)LL_TIM_GetCounter(TIM4);
  int16_t d   = (int16_t)(cnt - *prev);

  if (d >  (CAL_ENC_MODULO / 2)) { d = (int16_t)(d - CAL_ENC_MODULO); }
  if (d < -(CAL_ENC_MODULO / 2)) { d = (int16_t)(d + CAL_ENC_MODULO); }
  *prev = cnt;
  return d;
}

/**
 * @brief  Measures and applies the commutation offset, then restarts the
 *         MCSDK homing. Call from a task as soon as the motor is in RUN.
 * @retval CAL_OK if a new offset was applied, else a CAL_ERR_xxx code (the
 *         offset is then left at 0 and homing is still restarted).
 */
uint8_t commutation_calibrate(void)
{
  cal_acc_t acc[2] = {{0}};          /* [0] = + sweep, [1] = - sweep */
  int32_t   cnt[2] = {0, 0};
  int16_t   prev;
  int16_t   offset = 0;
  uint8_t   ok = 1U;
  uint8_t   status;
  int       s;
  int       i;

  if (RUN != MCI_GetSTMState(pMCI[M1]))
  {
    g_cal_status = CAL_ERR_NOT_RUN;
    printf("[CAL] skipped: motor not in RUN (state=%d)\r\n", (int)MCI_GetSTMState(pMCI[M1]));
    return CAL_ERR_NOT_RUN;
  }

  /* 1. Pause the MCSDK Z search; index pulses must not set zero meanwhile */
  PosCtrlM1.PositionControlRegulation = DISABLE;
  PosCtrlM1.AlignmentStatus           = TC_AWAITING_FOR_ALIGNMENT;
  MC_ProgramTorqueRampMotor1_F(0.0f, 0U);

  /* 2. Lock the rotor to the field where it is, then sweep +1 / -1 el rev */
  g_force_angle    = g_el_raw_dbg;
  g_openloop_id    = (int16_t)(CAL_ID_A * CURRENT_CONV_FACTOR);
  g_force_angle_en = 1U;
  ok = diag_wait_ms(CAL_LATCH_MS);

  for (s = 0; (s < 2) && (0U != ok); s++)
  {
    prev = (int16_t)LL_TIM_GetCounter(TIM4);

    for (i = 0; (i < CAL_STEPS) && (0U != ok); i++)
    {
      g_force_angle = (int16_t)(g_force_angle + ((0 == s) ? CAL_STEP : -CAL_STEP));
      ok = diag_wait_ms(CAL_STEP_MS);
      cnt[s] += cal_enc_delta(&prev);

      if (i >= CAL_SKIP_STEPS)
      {
        cal_acc_add(&acc[s], (int16_t)(g_force_angle - g_el_raw_dbg));  /* field - encoder */
      }
    }
  }

  (void)open_loop_exit();             /* back to closed-loop FOC, Id ref 0.
                                       * A timeout is only a warning: the clear
                                       * stays requested and the MF hook still
                                       * serves it on its next tick. */

  /* 3. Evaluate and apply */
  status = (0U == ok) ? CAL_ERR_ABORTED
                      : cal_evaluate(&acc[0], &acc[1], cnt[0], cnt[1], &offset);
  if ((CAL_OK == status) && (0U != g_cal_apply))
  {
    /* Tell the MCSDK encoder the corrected angle, so SPD_GetElAngle() (and
     * every MCSDK user of it) is right at the source - the same public call
     * the MCSDK alignment uses. mechanical = electrical / pole pairs.
     * ENC_SetMecAngle() rewrites TIM4->CNT AND the stored hMecAngle together,
     * so ENC_CalcAngle()'s next delta is ~0 and wMecAngle (position) does not
     * jump; rounding is at most 1 count (0.35 deg). Interrupts are masked so
     * the FOC ISR cannot run ENC_CalcAngle() between read and write. */
    int16_t mec_corr = (int16_t)(offset / (int16_t)ENCODER_M1._Super.bElToMecRatio);

    __disable_irq();
    ENC_SetMecAngle(&ENCODER_M1, (int16_t)(ENCODER_M1._Super.hMecAngle + mec_corr));
    __enable_irq();

    /* The count rewrite looks like one fast step to the speed estimator.
     * Let one 1 kHz speed sample take it, then flush the speed history
     * (ENC_Clear only clears the speed buffer, as the MCSDK does after its
     * own alignment). */
    vTaskDelay(pdMS_TO_TICKS(3));
    ENC_Clear(&ENCODER_M1);
  }
  g_cal_offset = offset;
  g_cal_status = status;

  printf("[CAL] %s: offset=%d s16 (%ld deg) | fwd %ld deg, back %ld deg, spread +/-%ld/%ld deg | enc %+ld/%+ld counts\r\n",
         (CAL_OK != status) ? "REJECTED (offset left at 0)"
                            : ((0U != g_cal_apply) ? "APPLIED" : "MEASURED ONLY (not applied)"),
         (int)offset, ((long)offset * 360L) / 65536L,
         (acc[0].n > 0U) ? ((long)cal_acc_mean(&acc[0]) * 360L) / 65536L : 0L,
         (acc[1].n > 0U) ? ((long)cal_acc_mean(&acc[1]) * 360L) / 65536L : 0L,
         ((long)cal_acc_spread(&acc[0]) * 360L) / 65536L,
         ((long)cal_acc_spread(&acc[1]) * 360L) / 65536L,
         (long)cnt[0], (long)cnt[1]);
  if (CAL_OK != status)
  {
    printf("[CAL] reason code %u (1 not RUN, 2 fault/guard, 3 encoder did not follow, 4 too scattered)\r\n",
           (unsigned)status);
  }
  else if ((offset > 16384) || (offset < -16384))
  {
    /* Beyond +/-90 electrical degrees the torque sign reverses if the
     * offset is NOT corrected: position control would push the wrong way */
    printf("[CAL] WARNING: |offset| > 90 deg -> uncorrected commutation would reverse torque\r\n");
  }

  /* 4. Restart the standard MCSDK homing with corrected commutation.
   *    TC_MoveCommand() only accepts a command when the controller is idle. */
  if (RUN == MCI_GetSTMState(pMCI[M1]))
  {
    uint32_t t0 = HAL_GetTick();

    while ((TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()) &&
           ((HAL_GetTick() - t0) < 3000U))
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    TC_EncAlignmentCommand(&PosCtrlM1);   /* 1 rev in 2 s; first Z sets zero */
  }

  return status;
}
