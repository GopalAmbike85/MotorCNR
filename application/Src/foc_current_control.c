
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"
#include <stdint.h>
#include<stdio.h>

#include "foc_current_control.h"
#include "elec_angle.h"   /* shared forced-angle / debug globals */

/* 1 = negate the measured phase currents before Clarke. 0 on this board
 * (with the current-sense channel correction below). Volatile so it can be
 * changed from the debugger for tests. */
#define ISENSE_INVERT_DEFAULT    0U

volatile uint8_t g_i_sense_invert = ISENSE_INVERT_DEFAULT;

/** @brief s16 current counts -> milliamps, for readable logs. */
static long counts_to_ma(int16_t counts)
{
  return ((long)counts * 1000L) / (long)CURRENT_CONV_FACTOR;
}

/**
 * @brief  Prints one line of drive status for bench diagnosis. Call from a
 *         task (not the ISR). Values are snapshots and not taken atomically,
 *         which is fine for logging.
 * @param  tag  short prefix, e.g. "HOME" or "RUN".
 *
 * Fields: t = ms since boot, pos = MCSDK position (mrad), Iqref/Iq/Id = q/d
 * current reference and measured values (mA), Vq = q voltage (s16),
 * enc = raw TIM4 count,
 * align = AlignStatus_t (1 searching, 2 done, 5 error), st = MCI state
 * (6 RUN, 11 FAULT_OVER), i2t = 1 if throttled, flt = current faults.
 */
void FOC_LogStatus(const char *tag)
{
  printf("[%s] t=%lu pos=%ld mrad Iqref=%ld Iq=%ld Id=%ld mA Vq=%d enc=%u align=%d st=%d i2t=%u flt=0x%04X\r\n",
         tag,
         (unsigned long)HAL_GetTick(),
         (long)(MC_GetCurrentPosition1() * 1000.0f),
         counts_to_ma(FOCVars[M1].Iqdref.q),
         counts_to_ma(FOCVars[M1].Iqd.q),
         counts_to_ma(FOCVars[M1].Iqd.d),
         (int)FOCVars[M1].Vqd.q,
         (unsigned)LL_TIM_GetCounter(TIM4),
         (int)MC_GetAlignmentStatusMotor1(),
         (int)MCI_GetSTMState(pMCI[M1]),
         (unsigned)g_i2t_throttled,
         (unsigned)MC_GetCurrentFaultsMotor1());
}

/* ===========================================================================
 * Phase-current re-order switch (DIAGNOSTIC, 26-09-2026)
 * ===========================================================================
 * Bench run 1 (polarity self-test, angle 0, only Vd applied) measured
 * Id = -595, Iq = -1092 (expected Iq ~ 0). With the MCSDK's own Clarke/Park
 * (at angle 0: q = alpha, d = beta; beta = -(a + 2b)/sqrt3):
 *                   U       V       W
 *   expected        0   -1078   +1078
 *   measured    -1092   +1061     +31
 *
 * TWO wiring faults fit that single reading exactly:
 *   H1  rotated order:        ch U = +V, ch V = +W, ch W = +U
 *   H2  U/W swapped+inverted: ch U = -W, ch V = -V, ch W = -U
 * Runs 2-4 used the H1 correction. Its self-test passed (it only checks
 * 0 deg), but run 5 showed Id growing -21 -> -62 -> -266 mA with ZERO
 * reference while the rotor sat at 90 deg after alignment. Worked through
 * the MCSDK maths: with the H1 correction on an H2 board, the d-loop gain is
 * +1 at 0 deg but -1 (positive feedback) at 90 deg. That matches the bench,
 * and explains "both position-KP signs run away". So H2 is the likely fault.
 *
 * H2 correction (applied when g_i_phase_remap = 1):
 *   true U = -measured W = measured U + measured V
 *   true V = -measured V
 * On the run-1 numbers this gives (-31, -1061, +1092) vs expected
 * (0, -1078, +1078).
 *
 * THIS IS A TEST AID, NOT THE FIX. R3_1 picks which two phases to sample from
 * the PWM sector assuming channel U belongs to PWM phase U; with the order
 * wrong it may sample a phase at a bad moment at high modulation. Once
 * confirmed, swap the U and W current-sense channels in Workbench / CubeMX
 * (or the PCB), keep the inversion via g_i_sense_invert = 1, and set
 * ISENSE_PHASE_REMAP_DEFAULT back to 0.
 * ======================================================================== */
#define ISENSE_PHASE_REMAP_DEFAULT  1U

/* 1 = apply the re-order below. Volatile: can be toggled from the debugger. */
volatile uint8_t g_i_phase_remap = ISENSE_PHASE_REMAP_DEFAULT;

/** @brief Saturates an int32 to the symmetric int16 range. */
static int16_t sat16(int32_t v)
{
  if (v >  INT16_MAX) { v =  INT16_MAX; }
  if (v < -INT16_MAX) { v = -INT16_MAX; }
  return (int16_t)v;
}

/**
 * @brief  H2 correction: out.a = a + b, out.b = -b (see block comment above).
 * @param  in  currents as delivered by PWMC_GetPhaseCurrents() (a = U, b = V).
 * @retval corrected currents, saturated to int16.
 */
static ab_t isense_phase_remap(ab_t in)
{
  ab_t out;

  out.a = sat16((int32_t)in.a + (int32_t)in.b);   /* true U = -measured W */
  out.b = sat16(-(int32_t)in.b);                  /* true V = -measured V */
  return out;
}

/* 1 while the i2t limiter is holding Iq to NOMINAL_CURRENT. Read-only
 * diagnostic, watch it from the debugger or print it from a task. */
volatile uint8_t g_i2t_throttled = 0U;

/* Averaging window = 2^17 FOC samples = 131072 / 25 kHz = ~5.2 s. */
#define I2T_WINDOW_SHIFT  17

/**
 * @brief  i2t thermal limiter. Returns the Iq reference limit to apply.
 *
 * @param  iq, id  measured (filtered) q/d currents, s16 counts.
 * @retval Iq limit in s16 counts: IQMAX normally, NOMINAL_CURRENT while
 *         throttled.
 *
 * @details The lead-screw needs brief current BURSTS (up to IQMAX) to break
 *          through its tight spots, but sustained high current overheats the
 *          small motor. Copper heating goes as I^2, so this tracks a running
 *          average of |I|^2 = Iq^2 + Id^2 (bug 7 fix; the old version averaged
 *          |Iq| only, which under-counts bursts and ignores Id entirely).
 *
 *          Hysteresis (also bug 7): throttle to NOMINAL_CURRENT once the
 *          average reaches NOMINAL^2; release to IQMAX only when it has fallen
 *          below (0.8 * NOMINAL)^2. The old code had a single threshold, so it
 *          flipped between the two limits roughly once a second.
 *
 *          Burst budget from cold at IQMAX = 0.8 A, NOMINAL = 0.356 A:
 *          t = -tau * ln(1 - (0.356/0.8)^2) = 5.24 s * 0.22 = ~1.2 s.
 *          Cool-down to release at zero current:
 *          t = tau * ln(1 / 0.64) = ~2.3 s.
 *
 *          Fixed point: acc = avg << 17 in int64, so the average keeps its
 *          fraction (no truncation bias). |I|^2 <= 2 * 32768^2 = 2.1e9 and
 *          acc <= 2.1e9 << 17 = 2.8e14, both far inside int64.
 *
 * @note   While throttled with Id != 0, total |I| is slightly above nominal
 *         (Iq is limited to nominal, not sqrt(nominal^2 - Id^2)). In normal
 *         running Id = 0, so this is accepted to avoid a sqrt in the ISR.
 */
static int16_t i2t_update(int16_t iq, int16_t id)
{
  /* Thresholds in counts^2, folded at compile time */
  static const int64_t k_trip    = (int64_t)((double)NOMINAL_CURRENT * (double)NOMINAL_CURRENT);
  static const int64_t k_release = (int64_t)(0.64 * (double)NOMINAL_CURRENT * (double)NOMINAL_CURRENT);
  static int64_t acc = 0;              /* average |I|^2, scaled by 2^17 */
  static uint8_t throttled = 0U;

  int64_t i2  = ((int64_t)iq * iq) + ((int64_t)id * id);
  int64_t avg;

  acc += i2 - (acc >> I2T_WINDOW_SHIFT);   /* avg += (i2 - avg) / 2^17 */
  avg  = acc >> I2T_WINDOW_SHIFT;

  if ((0U == throttled) && (avg >= k_trip))
  {
    throttled = 1U;
  }
  else if ((0U != throttled) && (avg < k_release))
  {
    throttled = 0U;
  }
  else
  {
    /* inside the hysteresis band: keep the current state */
  }

  g_i2t_throttled = throttled;

  return (0U != throttled) ? (int16_t)NOMINAL_CURRENT : (int16_t)IQMAX;
}

/**
 * @brief  Application FOC current controller (replaces the MCSDK's
 *         FOC_CurrControllerM1() at run time; see FOC_HighFrequencyTask()
 *         at the end of this file for how it is hooked in).
 * @note   Renamed from FOC_CurrControllerM1 so it no longer clashes with the
 *         generated MCSDK function, which is now left untouched in
 *         mc_tasks_foc.c and survives CubeMX / Workbench regeneration.
 */
uint16_t APP_FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;
  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle = SPD_GetElAngle(speedHandle);
  /* Capture the raw electrical angle for logging by reusing the value just
   * fetched, rather than calling SPD_GetElAngle() again. The FOC runs every
   * 2nd PWM period (50 kHz / REGULATION_EXECUTION_RATE 2 = 25 kHz), so it must
   * finish within 40 us or R3_1 returns MC_DURATION; keep the hot path lean. */
  g_el_raw_dbg = hElAngle;   /* raw pre-offset angle */
  /* NOTE: an earlier comment here said a PWM-noise glitch filter lives in
   * ENC_CalcAngle(). The compiled MCSDK encoder_speed_pos_fdbk.c is stock
   * and has no such filter (checked 26-09-2026). */
  /* No commutation offset is added here. commutation_calibrate()
   * (elec_angle.c) measures it at start-up and corrects the MCSDK encoder
   * itself with ENC_SetMecAngle(), so SPD_GetElAngle() above is already
   * right (26-09-2026). */
  /* Optional angle-override hook. When enabled, forces the electrical angle
   * to a known value so that applying Id locks the rotor to that angle. Left
   * disabled (g_force_angle_en = 0) in normal operation. */
  if (g_force_angle_en)
  {
    hElAngle = g_force_angle;
    /* Open-loop commutation test: drive a fixed d-axis current at the
     * FORCED electrical angle so the rotor aligns to and follows the field
     * (the encoder is not used for control here). This overrides the normal
     * current reference only while the forced-angle override is active;
     * g_openloop_id defaults to 0, so normal builds are unaffected. */
    FOCVars[M1].Iqdref.d = g_openloop_id;
    FOCVars[M1].Iqdref.q = 0;
  }

  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);

  /* Diagnostic phase re-order (see isense_phase_remap()). Applied before
   * everything else so the PI, i2t and logs all see it. */
  if (0U != g_i_phase_remap)
  {
    Iab = isense_phase_remap(Iab);
  }

  /* ---- PHASE-CURRENT SENSE POLARITY -----------------------------------------
   * MCSDK's R3_1 driver computes  Ia = PhaseAOffset - ADC, i.e. it assumes an
   * INVERTING sense chain (the ADC reading falls as positive phase current rises).
   * The DRV8316's on-chip CSA uses the opposite convention, so every measured current
   * arrives negated and the current PI becomes POSITIVE feedback.
   *
   * The evidence only became decisive once the current-loop integrator was enabled.
   * A P-ONLY loop is supposed to have large steady-state error, and at the ~0.04 A it
   * was delivering the measured sign was dominated by offset and noise - readings
   * contradicted each other run to run. With KI enabled an integrating loop MUST
   * drive measured current to its reference if the feedback sign is right. Instead
   * the bench showed
   *     Iqref = +155,  Iq = -198
   * - opposite sign AND larger magnitude, i.e. the loop diverging. That is only
   * possible with inverted feedback, and it explains why BOTH position-loop KP signs
   * ran the rotor away: the torque produced is not controlled by the sign of Iqref.
   *
   * Corrected at the single point where the measurement enters the control path, so
   * Clarke/Park, the current PI, the i2t limiter and every reported Iqd all see the
   * corrected sign. g_i_sense_invert is defined at the top of this file and is
   * volatile so it can be flipped from the debugger to A/B test without
   * rebuilding. */
  {
    if (0U != g_i_sense_invert)
    {
      Iab.a = (int16_t)(-Iab.a);
      Iab.b = (int16_t)(-Iab.b);
    }
  }

  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);

  /* Low-pass filter on the measured Iqd before the current PI. The raw Iqd
   * carries ADC/switching noise that would become torque ripple (audible
   * buzz); this first-order IIR (y += (x - y) >> N) smooths it. N is the
   * shift; larger N = heavier filtering. N=7 (~5.1 ms at 25 kHz) MATCHES SimpleFOC's
   * LPF_current.Tf = 0.005, which is what lets its P-only current loop run at high
   * (burst) current without oscillating/railing. We had under-filtered (N=5..6).
   *
   * Fixed-point form (bug 6 fix): the old  y += (x - y) >> 7  threw away the
   * fractional part every step, so y stopped moving whenever 0 <= x - y < 128.
   * That left a one-sided error of up to 127 counts (~43 mA with
   * CURRENT_CONV_FACTOR ~2978 counts/A), and only when approaching from below.
   * The state is now kept scaled by 2^N (acc = y << N) so the fraction is
   * retained, and the output is rounded to nearest. Same time constant and
   * same loop dynamics as before; residual error is at most 0.5 count.
   * Range: |acc| <= 32768 << 7 = 4.2M, well inside int32.
   * Note: >> on a negative int32 is an arithmetic shift on GCC/ARM. */
  #define FOC_IQD_LPF_SHIFT  7
  {
    static int32_t iq_acc = 0, id_acc = 0;          /* filtered value << N */
    const int32_t  half   = (int32_t)1 << (FOC_IQD_LPF_SHIFT - 1);
    int32_t iq_y = (iq_acc + half) >> FOC_IQD_LPF_SHIFT;   /* rounded output */
    int32_t id_y = (id_acc + half) >> FOC_IQD_LPF_SHIFT;

    iq_acc += (int32_t)Iqd.q - iq_y;                /* acc += x - y  ==  y += (x - y) / 2^N */
    id_acc += (int32_t)Iqd.d - id_y;

    Iqd.q = (int16_t)((iq_acc + half) >> FOC_IQD_LPF_SHIFT);
    Iqd.d = (int16_t)((id_acc + half) >> FOC_IQD_LPF_SHIFT);
  }
  /* i2t torque-current limiter - see i2t_update() above for the model. */
  {
    int16_t i2t_lim = i2t_update(Iqd.q, Iqd.d);
    if (FOCVars[M1].Iqdref.q >  i2t_lim) { FOCVars[M1].Iqdref.q =  i2t_lim; }
    if (FOCVars[M1].Iqdref.q < -i2t_lim) { FOCVars[M1].Iqdref.q = -i2t_lim; }
  }
  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    Vqd.q = PI_Controller(pPIDIq[M1], (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M1], (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }
  Vqd = Circle_Limitation(&CircleLimitationM1, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }
  else
  {
    /* Nothing to do. No PWM setting to prevent possible ChargeBootCap conflict */

  }
  // printf("FOC_CurrControllerM1\n");

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  return (hCodeError);
}

/**
 * @brief  Strong override of the MCSDK's __weak FOC_HighFrequencyTask()
 *         (mc_tasks_foc.c). Called from TSK_HighFrequencyTask() in the ADC
 *         interrupt on every FOC tick.
 *
 * @details Same sequence as the generated version, except that it calls
 *          APP_FOC_CurrControllerM1() instead of the MCSDK's
 *          FOC_CurrControllerM1(). The linker picks this strong definition
 *          over the weak one, so mc_tasks_foc.c no longer has to be edited
 *          and can be regenerated freely.
 *
 *          KEEP IN SYNC: if a future MCSDK / Workbench regeneration changes
 *          the body of the weak FOC_HighFrequencyTask() (e.g. different
 *          sensor, extra RCM calls), mirror that change here.
 *          Generated reference: MCSDK 6.4.2, Src/mc_tasks_foc.c.
 *
 * @param  bMotorNbr  motor index (single drive: always M1).
 * @retval bMotorNbr
 */
uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;

  RCM_ReadOngoingConv();                 /* regular ADC conversions (Vbus, temp) */
  RCM_ExecNextConv();

  (void)ENC_CalcAngle(&ENCODER_M1);      /* encoder angle for this tick */

  hFOCreturn = APP_FOC_CurrControllerM1();

  if (hFOCreturn == MC_DURATION)
  {
    /* FOC did not finish before the next PWM update: same fault as MCSDK */
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }

  return (bMotorNbr);
}

/* ===========================================================================
 * Software runaway guard (26-09-2026)
 * ===========================================================================
 * WHY: on the bench, a wrong position-loop sign drove the rotor ~605 revs in
 * 5 s at full torque. This drive has no hardware overcurrent protection, so
 * a software stop is the only line of defence.
 *
 * WHAT: runs after every MCSDK medium-frequency task (1 kHz, position loop
 * rate). While the motor is in RUN it stops the drive (MC_SW_ERROR -> PWM off
 * via the MCSDK safety task) if
 *   - |mechanical speed| > GUARD_MAX_RPM        (any RUN mode: position,
 *                                                torque, open-loop tests), or
 *   - |position reference - position| > GUARD_MAX_ERR_RAD
 *                                               (only while position
 *                                                regulation is enabled)
 * for GUARD_DEBOUNCE_TICKS consecutive ticks (so one encoder glitch cannot
 * trip it). The reason and values are latched for FOC_GuardReport().
 * ======================================================================== */
#define GUARD_MAX_RPM          50000L   /* user choice (500 -> 5000 -> 50000). Equals MOTOR_MAX_SPEED_RPM, so the over-speed check is effectively OFF; the position-error check still protects position moves */
#define GUARD_MAX_ERR_RAD      12.566f  /* 2 revolutions = 4*pi rad              */
#define GUARD_DEBOUNCE_TICKS   5U       /* 5 ms at the 1 kHz medium-task rate     */

#define GUARD_OK         0U
#define GUARD_TRIP_SPEED 1U
#define GUARD_TRIP_ERROR 2U

volatile uint8_t g_guard_trip     = GUARD_OK;  /* latched reason, 0 = none */
volatile int32_t g_guard_rpm      = 0;         /* speed at the trip (rpm)  */
volatile int32_t g_guard_err_mrad = 0;         /* pos error at trip (mrad) */

/**
 * @brief  Pure guard logic (no MCSDK calls, unit-testable on a PC).
 * @param  active     1 if the guard should be checking (motor in RUN).
 * @param  check_err  1 to also check position error (regulation enabled).
 * @param  rpm        mechanical speed, rpm.
 * @param  err_rad    position reference minus position, rad.
 * @retval GUARD_OK, or the trip reason once the fault has persisted for
 *         GUARD_DEBOUNCE_TICKS calls. Counters reset whenever inactive.
 */
static uint8_t guard_step(uint8_t active, uint8_t check_err, int32_t rpm, float err_rad)
{
  static uint8_t s_speed_cnt = 0U;
  static uint8_t s_err_cnt   = 0U;
  uint8_t result = GUARD_OK;

  if (0U == active)
  {
    s_speed_cnt = 0U;
    s_err_cnt   = 0U;
  }
  else
  {
    int32_t rpm_abs = (rpm < 0) ? -rpm : rpm;
    float   err_abs = (err_rad < 0.0f) ? -err_rad : err_rad;

    s_speed_cnt = (rpm_abs > GUARD_MAX_RPM) ? (uint8_t)(s_speed_cnt + 1U) : 0U;
    s_err_cnt   = ((0U != check_err) && (err_abs > GUARD_MAX_ERR_RAD))
                  ? (uint8_t)(s_err_cnt + 1U) : 0U;

    if (s_speed_cnt >= GUARD_DEBOUNCE_TICKS)
    {
      result = GUARD_TRIP_SPEED;
    }
    else if (s_err_cnt >= GUARD_DEBOUNCE_TICKS)
    {
      result = GUARD_TRIP_ERROR;
    }
    else
    {
      /* within limits, or not yet persistent */
    }
  }

  return result;
}

/**
 * @brief  Strong override of the MCSDK's __weak hook (mc_app_hooks.c), called
 *         right after TSK_MediumFrequencyTaskM1() in the medium-frequency task.
 */
void MC_APP_PostMediumFrequencyHook_M1(void)
{
  uint8_t active    = ((RUN == Mci[M1].State) &&
                       (GUARD_OK == g_guard_trip)) ? 1U : 0U;
  uint8_t check_err = (ENABLE == PosCtrlM1.PositionControlRegulation) ? 1U : 0U;
  int32_t rpm     = 0;
  float   err_rad = 0.0f;
  uint8_t trip;

  /* Race-free Id clear requested by open_loop_exit() (elec_angle.c).
   * This hook runs in the MF task right AFTER TSK_MediumFrequencyTaskM1()
   * -> FOC_CalcCurrRef() has written back its Iqdref copy, so a stale Id
   * from a pre-empted FOC_CalcCurrRef() is overwritten here and cannot come
   * back. Skipped while open loop is (re-)enabled, as the ISR owns .d then.
   * Interrupts masked: the FOC ISR also writes Iqdref. */
  if ((0U != g_id_clear_req) && (0U == g_force_angle_en))
  {
    __disable_irq();
    FOCVars[M1].Iqdref.d = 0;
    g_id_clear_req       = 0U;
    __enable_irq();
  }

  if (0U != active)
  {
    /* 32-bit conversion: SPEED_UNIT_2_RPM() casts to int16_t and would wrap
     * above 32767 rpm, so a 50000 rpm limit could never be compared */
    rpm = ((int32_t)MC_GetMecSpeedAverageMotor1() * (int32_t)U_RPM) / (int32_t)SPEED_UNIT;
    if (0U != check_err)
    {
      err_rad = TC_GetCtrlPositionAngle(&PosCtrlM1) - TC_GetCurrentPosition(&PosCtrlM1);
    }
  }

  trip = guard_step(active, check_err, rpm, err_rad);

  if (GUARD_OK != trip)
  {
    g_guard_rpm      = rpm;
    g_guard_err_mrad = (int32_t)(err_rad * 1000.0f);
    g_guard_trip     = trip;                             /* latch */
    MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0U);      /* PWM off */
  }
}

/**
 * @brief  Prints the guard trip reason once. Call from a task loop.
 */
void FOC_GuardReport(void)
{
  static uint8_t s_reported = 0U;

  if ((GUARD_OK != g_guard_trip) && (0U == s_reported))
  {
    s_reported = 1U;
    printf("[GUARD] STOPPED: %s. speed=%ld rpm, position error=%ld mrad (limits %ld rpm, %ld mrad)\r\n",
           (GUARD_TRIP_SPEED == g_guard_trip) ? "over-speed" : "position error too large",
           (long)g_guard_rpm, (long)g_guard_err_mrad,
           (long)GUARD_MAX_RPM, (long)(GUARD_MAX_ERR_RAD * 1000.0f));
  }
}
