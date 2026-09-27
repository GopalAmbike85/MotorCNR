#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"
#include "mc_interface.h"

#include "app_foc_task.h"
#include "elec_angle.h"
#include "foc_current_control.h"
#include "app_time.h"

#define APP_DIAG_COMMUTATION  0

#define APP_MOVE_TARGET_RAD   100.0f
#define APP_MOVE_DURATION_S   10.0f

static void app_stop(const char *reason)
{
  printf("[STOP] %s. Motor stopped (PWM off).\r\n", reason);
  (void)MC_StopMotor1();
}

static void app_position_move_test(void)
{
  if (RUN != MCI_GetSTMState(pMCI[M1]))
  {
    printf("[MOVE] skipped: motor not in RUN (state=%d)\r\n", (int)MCI_GetSTMState(pMCI[M1]));
  }
  else
  {
    TickType_t t0 = xTaskGetTickCount();
    TickType_t t_log;

    while ((TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1()) && ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(300)))
    {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (TC_READY_FOR_COMMAND != MC_GetControlPositionStatusMotor1())
    {
      printf("[MOVE] skipped: position controller not ready (status=%d)\r\n",
             (int)MC_GetControlPositionStatusMotor1());
    }
    else
    {
      printf("[MOVE] start: from %ld mrad to %ld mrad in %ld ms (align=%d)\r\n",
             (long)(MC_GetCurrentPosition1() * 1000.0f),
             (long)(APP_MOVE_TARGET_RAD * 1000.0f),
             (long)(APP_MOVE_DURATION_S * 1000.0f),
             (int)MC_GetAlignmentStatusMotor1());

      PID_SetKP(&PID_PosParamsM1, 500);
      PID_SetKI(&PID_PosParamsM1, 0);
      PID_SetKD(&PID_PosParamsM1, 0);
      PID_SetPrevError(&PID_PosParamsM1, 0);
      PID_SetIntegralTerm(&PID_PosParamsM1, 0);

      MC_ProgramPositionCommandMotor1(APP_MOVE_TARGET_RAD, APP_MOVE_DURATION_S);

      t0 = xTaskGetTickCount();
      while ((xTaskGetTickCount() - t0) <= pdMS_TO_TICKS((uint32_t)(APP_MOVE_DURATION_S * 1000.0f) + 1000U))
      {
        vTaskDelay(pdMS_TO_TICKS(20));
      }

      t0    = xTaskGetTickCount();
      t_log = t0;
      while (((xTaskGetTickCount() - t0) < pdMS_TO_TICKS((uint32_t)(APP_MOVE_DURATION_S * 1000.0f) + 2000U)) &&
             (RUN == MCI_GetSTMState(pMCI[M1])))
      {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - t_log) >= pdMS_TO_TICKS(200))
        {
          t_log = xTaskGetTickCount();
        }
      }

      printf("[MOVE] end: position %ld mrad (target %ld mrad), status=%d, state=%d\r\n",
             (long)(MC_GetCurrentPosition1() * 1000.0f),
             (long)(APP_MOVE_TARGET_RAD * 1000.0f),
             (int)MC_GetControlPositionStatusMotor1(),
             (int)MCI_GetSTMState(pMCI[M1]));
    }
  }
}

static void app_wait_z_homing(void)
{
  TickType_t    z_start    = xTaskGetTickCount();
  TickType_t    z_last_log = z_start;
  AlignStatus_t z_status   = MC_GetAlignmentStatusMotor1();

  while ((TC_ALIGNMENT_COMPLETED != z_status) && (TC_ALIGNMENT_ERROR != z_status) &&
         ((xTaskGetTickCount() - z_start) < pdMS_TO_TICKS(6000)))
  {
    vTaskDelay(pdMS_TO_TICKS(5));
    z_status = MC_GetAlignmentStatusMotor1();

    FOC_GuardReport();

    if ((xTaskGetTickCount() - z_last_log) >= pdMS_TO_TICKS(100))
    {
      z_last_log = xTaskGetTickCount();
    }
  }

  if (TC_ALIGNMENT_COMPLETED == z_status)
  {
    printf("[HOME] Z index found by MCSDK. Position zero set.\r\n");
    app_position_move_test();
  }
  else if (TC_ALIGNMENT_ERROR == z_status)
  {
    printf("[HOME] FAIL: MCSDK 1-rev search ended with no Z pulse (TC_ALIGNMENT_ERROR).\r\n");
    printf("[HOME]       Check Z wiring on PB6 and that the rotor actually turned.\r\n");
    printf("[MOVE] skipped: homing failed\r\n");
    app_stop("Homing failed");
  }
  else
  {
    printf("[HOME] FAIL: timeout, alignment status=%d, state=%d\r\n",
           (int)z_status, (int)MCI_GetSTMState(pMCI[M1]));
    printf("[MOVE] skipped: homing failed\r\n");
    app_stop("Homing timed out");
  }
}

void app_foc_task_run(void const *argument)
{
  TickType_t start_tick;
  uint8_t    app_ok = 1U;

  (void)argument;

  MC_ProgramTorqueRampMotor1(0, 0U);

  PosCtrlM1.AlignmentStatus = TC_ALIGNMENT_COMPLETED;

  MC_StartMotor1();
  start_tick = xTaskGetTickCount();

  while (MCI_GetSTMState(pMCI[M1]) != RUN)
  {
    if ((xTaskGetTickCount() - start_tick) >= pdMS_TO_TICKS(6000))
    {
      printf("[FOC] Timeout waiting for motor RUN state\r\n");
      app_stop("RUN state not reached");
      app_ok = 0U;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }

#if (APP_DIAG_COMMUTATION != 1)

  if (0U != app_ok)
  {
    if (0U != commutation_calibrate())
    {
      FOC_GuardReport();
      app_stop("Commutation calibration failed");
      app_ok = 0U;
    }
  }
  FOC_GuardReport();

#endif

  for (;;)
  {
    FOC_GuardReport();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
