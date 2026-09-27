#include "main.h"

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
#include "elec_angle.h"

#define ISENSE_INVERT_DEFAULT    0U

volatile uint8_t g_i_sense_invert = ISENSE_INVERT_DEFAULT;

static long counts_to_ma(int16_t counts)
{
  return ((long)counts * 1000L) / (long)CURRENT_CONV_FACTOR;
}

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

#define ISENSE_PHASE_REMAP_DEFAULT  1U

volatile uint8_t g_i_phase_remap = ISENSE_PHASE_REMAP_DEFAULT;

static int16_t sat16(int32_t v)
{
  if (v >  INT16_MAX) { v =  INT16_MAX; }
  if (v < -INT16_MAX) { v = -INT16_MAX; }
  return (int16_t)v;
}

static ab_t isense_phase_remap(ab_t in)
{
  ab_t out;

  out.a = sat16((int32_t)in.a + (int32_t)in.b);
  out.b = sat16(-(int32_t)in.b);
  return out;
}

volatile uint8_t g_i2t_throttled = 0U;

#define I2T_WINDOW_SHIFT  17

static int16_t i2t_update(int16_t iq, int16_t id)
{
  static const int64_t k_trip    = (int64_t)((double)NOMINAL_CURRENT * (double)NOMINAL_CURRENT);
  static const int64_t k_release = (int64_t)(0.64 * (double)NOMINAL_CURRENT * (double)NOMINAL_CURRENT);
  static int64_t acc = 0;
  static uint8_t throttled = 0U;

  int64_t i2  = ((int64_t)iq * iq) + ((int64_t)id * id);
  int64_t avg;

  acc += i2 - (acc >> I2T_WINDOW_SHIFT);
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
  }

  g_i2t_throttled = throttled;

  return (0U != throttled) ? (int16_t)NOMINAL_CURRENT : (int16_t)IQMAX;
}

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

  g_el_raw_dbg = hElAngle;

  if (g_force_angle_en)
  {
    hElAngle = g_force_angle;

    FOCVars[M1].Iqdref.d = g_openloop_id;
    FOCVars[M1].Iqdref.q = 0;
  }

  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);

  if (0U != g_i_phase_remap)
  {
    Iab = isense_phase_remap(Iab);
  }

  {
    if (0U != g_i_sense_invert)
    {
      Iab.a = (int16_t)(-Iab.a);
      Iab.b = (int16_t)(-Iab.b);
    }
  }

  Ialphabeta = MCM_Clarke(Iab);
  Iqd = MCM_Park(Ialphabeta, hElAngle);

  #define FOC_IQD_LPF_SHIFT  7
  {
    static int32_t iq_acc = 0, id_acc = 0;
    const int32_t  half   = (int32_t)1 << (FOC_IQD_LPF_SHIFT - 1);
    int32_t iq_y = (iq_acc + half) >> FOC_IQD_LPF_SHIFT;
    int32_t id_y = (id_acc + half) >> FOC_IQD_LPF_SHIFT;

    iq_acc += (int32_t)Iqd.q - iq_y;
    id_acc += (int32_t)Iqd.d - id_y;

    Iqd.q = (int16_t)((iq_acc + half) >> FOC_IQD_LPF_SHIFT);
    Iqd.d = (int16_t)((id_acc + half) >> FOC_IQD_LPF_SHIFT);
  }

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
  }

  FOCVars[M1].Vqd = Vqd;
  FOCVars[M1].Iab = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle = hElAngle;

  return (hCodeError);
}

uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;

  RCM_ReadOngoingConv();
  RCM_ExecNextConv();

  (void)ENC_CalcAngle(&ENCODER_M1);

  hFOCreturn = APP_FOC_CurrControllerM1();

  if (hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }

  return (bMotorNbr);
}

#define GUARD_MAX_RPM          50000L
#define GUARD_MAX_ERR_RAD      12.566f
#define GUARD_DEBOUNCE_TICKS   5U

#define GUARD_OK         0U
#define GUARD_TRIP_SPEED 1U
#define GUARD_TRIP_ERROR 2U

volatile uint8_t g_guard_trip     = GUARD_OK;
volatile int32_t g_guard_rpm      = 0;
volatile int32_t g_guard_err_mrad = 0;

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
    }
  }

  return result;
}

void MC_APP_PostMediumFrequencyHook_M1(void)
{
  uint8_t active    = ((RUN == Mci[M1].State) &&
                       (GUARD_OK == g_guard_trip)) ? 1U : 0U;
  uint8_t check_err = (ENABLE == PosCtrlM1.PositionControlRegulation) ? 1U : 0U;
  int32_t rpm     = 0;
  float   err_rad = 0.0f;
  uint8_t trip;

  if ((0U != g_id_clear_req) && (0U == g_force_angle_en))
  {
    __disable_irq();
    FOCVars[M1].Iqdref.d = 0;
    g_id_clear_req       = 0U;
    __enable_irq();
  }

  if (ZS_REQ_PENDING == g_zsearch_req)
  {
    if (RUN == Mci[M1].State)
    {
      PosCtrlM1.AlignmentStatus = TC_AWAITING_FOR_ALIGNMENT;
      TC_EncAlignmentCommand(&PosCtrlM1);
      g_zsearch_req = ZS_REQ_DONE;
    }
    else
    {
      g_zsearch_req = ZS_REQ_DROPPED;
    }
  }

  if (0U != active)
  {
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
    g_guard_trip     = trip;
    MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0U);
  }
}

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
