#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "mc_type.h"

#include "elec_angle.h"
#include "app_time.h"

volatile uint8_t g_force_angle_en = 0;
volatile int16_t g_force_angle    = 0;
volatile int16_t g_el_raw_dbg     = 0;
volatile int16_t g_openloop_id    = 0;
volatile uint8_t g_id_clear_req   = 0U;
volatile uint8_t g_zsearch_req    = ZS_REQ_IDLE;

#define ZS_START_CONFIRM_MS  50U

#define OL_EXIT_CONFIRM_MS   20U

static uint8_t open_loop_exit(void)
{
  uint32_t t0;
printf("[OL] exit: clearing Id and disabling forced angle\r\n");
  __disable_irq();
  g_force_angle_en        = 0U;
  g_openloop_id           = 0;
  FOCVars[M1].Iqdref.d    = 0;
  g_id_clear_req          = 1U;
  __enable_irq();

  t0 = HAL_GetTick();
  while ((0U != g_id_clear_req) && ((HAL_GetTick() - t0) < OL_EXIT_CONFIRM_MS))
  {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (0U != g_id_clear_req)
  {
    printf("[OL] WARNING: Id clear not confirmed by MF task within %u ms (still pending)\r\n",
           (unsigned)OL_EXIT_CONFIRM_MS);
    return 0U;
  }
  return 1U;
}

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

#define CAL_ID_A          0.30f
#define CAL_STEPS         100
#define CAL_STEP          655
#define CAL_STEP_MS       20U
#define CAL_LATCH_MS      300U
#define CAL_SKIP_STEPS    10

#define CAL_MIN_COUNTS    900
#define CAL_MAX_SPREAD    5461
#define CAL_ENC_MODULO    1024

#define CAL_OK            0U
#define CAL_ERR_NOT_RUN   1U
#define CAL_ERR_ABORTED   2U
#define CAL_ERR_FOLLOW    3U
#define CAL_ERR_SPREAD    4U

volatile uint8_t g_cal_status = 0xFFU;
volatile int16_t g_cal_offset = 0;

volatile uint8_t g_cal_apply  = 1U;

typedef struct
{
  int16_t  ref;
  int32_t  sum;
  int16_t  dmin;
  int16_t  dmax;
  uint16_t n;
} cal_acc_t;

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

static int16_t cal_acc_mean(const cal_acc_t *a)
{
  return (int16_t)(a->ref + (int16_t)(a->sum / (int32_t)a->n));
}

static int16_t cal_acc_spread(const cal_acc_t *a)
{
  return (int16_t)(((int32_t)a->dmax - (int32_t)a->dmin) / 2);
}

static uint8_t cal_evaluate(const cal_acc_t *fwd, const cal_acc_t *back,
                            int32_t cnt_fwd, int32_t cnt_back, int16_t *offset_out)
{
  int16_t m_f;
  int16_t m_b;

  if ((0U == fwd->n) || (0U == back->n) ||
      (cnt_fwd < CAL_MIN_COUNTS) || (cnt_back > -CAL_MIN_COUNTS))
  {
    return CAL_ERR_FOLLOW;
  }
  if ((cal_acc_spread(fwd) > CAL_MAX_SPREAD) || (cal_acc_spread(back) > CAL_MAX_SPREAD))
  {
    return CAL_ERR_SPREAD;
  }

  m_f = cal_acc_mean(fwd);
  m_b = cal_acc_mean(back);
  *offset_out = (int16_t)(m_f + (int16_t)((int16_t)(m_b - m_f) / 2));
  return CAL_OK;
}

static int16_t cal_enc_delta(int16_t *prev)
{
  int16_t cnt = (int16_t)LL_TIM_GetCounter(TIM4);
  int16_t d   = (int16_t)(cnt - *prev);

  if (d >  (CAL_ENC_MODULO / 2)) { d = (int16_t)(d - CAL_ENC_MODULO); }
  if (d < -(CAL_ENC_MODULO / 2)) { d = (int16_t)(d + CAL_ENC_MODULO); }
  *prev = cnt;
  return d;
}

#define CAL_LOG_EVERY     1

static long cal_deg(int16_t a)
{
  return ((long)a * 360L) / 65536L;
}

static long cal_ma(int16_t counts)
{
  return ((long)counts * 1000L) / (long)CURRENT_CONV_FACTOR;
}

static void cal_log_state(const char *tag)
{
  printf("[SWEEP] %s: t=%lu field=%ld deg enc=%ld deg diff=%ld deg tim4=%u Id=%ld Iq=%ld mA st=%d\r\n",
         tag, (unsigned long)HAL_GetTick(),
         cal_deg(g_force_angle), cal_deg(g_el_raw_dbg),
         cal_deg((int16_t)(g_force_angle - g_el_raw_dbg)),
         (unsigned)LL_TIM_GetCounter(TIM4),
         cal_ma(FOCVars[M1].Iqd.d), cal_ma(FOCVars[M1].Iqd.q),
         (int)MCI_GetSTMState(pMCI[M1]));
}

static void cal_log_step(int s, int i, int32_t cnt)
{
#if (CAL_LOG_EVERY > 0)
  if (0 == (i % CAL_LOG_EVERY))
  {
    printf("[SWEEP],%c,%d,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d\r\n",
           (0 == s) ? '+' : '-', i,
           cal_deg(g_force_angle), cal_deg(g_el_raw_dbg),
           cal_deg((int16_t)(g_force_angle - g_el_raw_dbg)),
           (long)cnt,
           cal_ma(FOCVars[M1].Iqd.d), cal_ma(FOCVars[M1].Iqd.q),
           (int)FOCVars[M1].Vqd.d, (int)FOCVars[M1].Vqd.q,
           (i >= CAL_SKIP_STEPS) ? 1 : 0);
  }
#else
  (void)s; (void)i; (void)cnt;
#endif
}

static void cal_log_sweep_end(int s, int32_t cnt, const cal_acc_t *a)
{
  printf("[SWEEP] %s sweep done: enc %+ld counts (need %s%d) | %u samples, mean %ld deg, spread +/-%ld deg (max %ld)\r\n",
         (0 == s) ? "forward" : "backward", (long)cnt,
         (0 == s) ? ">= +" : "<= -", (int)CAL_MIN_COUNTS,
         (unsigned)a->n,
         (a->n > 0U) ? cal_deg(cal_acc_mean(a)) : 0L,
         cal_deg(cal_acc_spread(a)), cal_deg((int16_t)CAL_MAX_SPREAD));
}

uint8_t commutation_calibrate(void)
{
  cal_acc_t acc[2] = {{0}};
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

  PosCtrlM1.PositionControlRegulation = DISABLE;
  MC_ProgramTorqueRampMotor1_F(0.0f, 0U);

  g_force_angle    = g_el_raw_dbg;
  g_openloop_id    = (int16_t)(CAL_ID_A * CURRENT_CONV_FACTOR);
  cal_log_state("LOCK start");
  g_force_angle_en = 1U;
  ok = diag_wait_ms(CAL_LATCH_MS);
  cal_log_state("LOCK end");

#if (CAL_LOG_EVERY > 0)
  printf("[SWEEP],dir,step,field_deg,enc_deg,diff_deg,cnt,id_mA,iq_mA,vd,vq,rec\r\n");
#endif

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
        cal_acc_add(&acc[s], (int16_t)(g_force_angle - g_el_raw_dbg));
      }
      cal_log_step(s, i, cnt[s]);
    }
    cal_log_sweep_end(s, cnt[s], &acc[s]);
  }
  if (0U == ok)
  {
    cal_log_state("ABORTED");
  }

  (void)open_loop_exit();

  status = (0U == ok) ? CAL_ERR_ABORTED
                      : cal_evaluate(&acc[0], &acc[1], cnt[0], cnt[1], &offset);

  printf("[CAL] %s (NOT applied, test build): offset=%d s16 (%ld deg) | fwd %ld deg, back %ld deg, spread +/-%ld/%ld deg | enc %+ld/%+ld counts\r\n",
         (CAL_OK == status) ? "ACCEPTED" : "REJECTED",
         (int)offset, cal_deg(offset),
         (acc[0].n > 0U) ? cal_deg(cal_acc_mean(&acc[0])) : 0L,
         (acc[1].n > 0U) ? cal_deg(cal_acc_mean(&acc[1])) : 0L,
         cal_deg(cal_acc_spread(&acc[0])), cal_deg(cal_acc_spread(&acc[1])),
         (long)cnt[0], (long)cnt[1]);
  if (CAL_OK != status)
  {
    printf("[CAL] reason code %u (2 fault/guard during sweep, 3 encoder did not follow, 4 too scattered)\r\n",
           (unsigned)status);
  }

  return status;
}
