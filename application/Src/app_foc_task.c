/* ---------------------------------------------------------------------------
 * app_foc_task.c - application body of the foc_task thread (26-09-2026).
 *
 * Moved out of Src/main.c (USER CODE 4 and 5) unchanged in behaviour, except
 * for one fix at the end of app_foc_task_run(): a FreeRTOS task must never
 * return, and the old body did (see the note there).
 *
 * Sequence:
 *   1. MC_StartMotor1(): MCSDK ALIGNMENT, then RUN (6 s timeout).
 *   2. commutation_calibrate() (elec_angle.c): measures/applies the
 *      commutation offset, then restarts the MCSDK Z-index search.
 *   3. Wait for the MCSDK Z-index result (position zero), 6 s timeout.
 *   4. app_position_move_test(): absolute move from the new zero.
 * Any failure stops the motor (PWM off) and skips the remaining steps.
 * ------------------------------------------------------------------------- */
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>

#include "mc_api.h"
#include "mc_config.h"             /* pMCI[], PID_PosParamsM1 */
#include "mc_interface.h"          /* MCI_GetSTMState() */

#include "app_foc_task.h"
#include "elec_angle.h"            /* commutation_calibrate() */
#include "foc_current_control.h"   /* FOC_GuardReport(), FOC_LogStatus() */
#include "app_time.h"              /* pdMS_TO_TICKS(): real ms (FreeRTOS tick is 2 kHz) */

/* ---------------------------------------------------------------------------
 * Configuration (moved from main.c USER CODE PM)
 * ------------------------------------------------------------------------- */
#define APP_DIAG_COMMUTATION  0  /* diagnostic OFF: normal MCSDK homing (26-09-2026) */

/* Position move (app_position_move_test), run after homing succeeds (or
 * after the diagnostic when APP_DIAG_COMMUTATION = 1): absolute target in
 * rad from position zero (the Z index after homing) and move time in s. */
#define APP_MOVE_TARGET_RAD   100.0f
#define APP_MOVE_DURATION_S   10.0f

/**
 * @brief  Stops the motor (PWM off, no current) after a start-up failure, so
 *         that it is never left powered while unable to move.
 * @param  reason  short text printed with the [STOP] message.
 */
static void app_stop(const char *reason)
{
  printf("[STOP] %s. Motor stopped (PWM off).\r\n", reason);
  (void)MC_StopMotor1();
}

/**
 * @brief  Position move test: APP_MOVE_TARGET_RAD in APP_MOVE_DURATION_S.
 * @note   MC_ProgramPositionCommandMotor1() takes an ABSOLUTE target (rad) and
 *         starts from the current position. TC_MoveCommand() silently ignores
 *         the command unless the position controller is idle
 *         (TC_READY_FOR_COMMAND), so wait for that first. The runaway guard
 *         stays active; 100 rad in 10 s is ~95 rpm average, ~140 rpm peak.
 * @note   All waits use pdMS_TO_TICKS(), which gives real ms on this project
 *         (overridden in FreeRTOSConfig.h for the 2 kHz tick, see
 *         app_time.h). Before 26-09-2026 it gave HALF the real time, so the
 *         old waits were half as long as their comments said.
 */
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

    /* Wait for the position controller to become idle */
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

      /* Bench-proven position gains (KP 500, KI 0, KD 0); also clear the
       * position PID's memory so the move starts from a clean state. */
      PID_SetKP(&PID_PosParamsM1, 500);
      PID_SetKI(&PID_PosParamsM1, 0);
      PID_SetKD(&PID_PosParamsM1, 0);
      PID_SetPrevError(&PID_PosParamsM1, 0);
      PID_SetIntegralTerm(&PID_PosParamsM1, 0);

      MC_ProgramPositionCommandMotor1(APP_MOVE_TARGET_RAD, APP_MOVE_DURATION_S);

      /* Wait move time + 1 s. APP_MOVE_DURATION_S is float: cast the ms
       * value to uint32_t. */
      t0 = xTaskGetTickCount();
      while ((xTaskGetTickCount() - t0) <= pdMS_TO_TICKS((uint32_t)(APP_MOVE_DURATION_S * 1000.0f) + 1000U))
      {
        vTaskDelay(pdMS_TO_TICKS(20));
      }

      /* Then watch for the move time + 2 s, or until a fault */
      t0    = xTaskGetTickCount();
      t_log = t0;
      while (((xTaskGetTickCount() - t0) < pdMS_TO_TICKS((uint32_t)(APP_MOVE_DURATION_S * 1000.0f) + 2000U)) &&
             (RUN == MCI_GetSTMState(pMCI[M1])))
      {
        vTaskDelay(pdMS_TO_TICKS(10));
        if ((xTaskGetTickCount() - t_log) >= pdMS_TO_TICKS(200))
        {
          t_log = xTaskGetTickCount();
          /* FOC_LogStatus("MOVE"); */   /* enable for a 200 ms trace */
        }
      }

      /* rad -> mrad is x1000 */
      printf("[MOVE] end: position %ld mrad (target %ld mrad), status=%d, state=%d\r\n",
             (long)(MC_GetCurrentPosition1() * 1000.0f),   /* current position, mrad */
             (long)(APP_MOVE_TARGET_RAD * 1000.0f),        /* target position, mrad  */
             (int)MC_GetControlPositionStatusMotor1(),
             (int)MCI_GetSTMState(pMCI[M1]));
    }
  }
}

/**
 * @brief  Waits for the MCSDK Z-index search (restarted by
 *         commutation_calibrate()) and reports the result. On success runs
 *         the position move; on failure stops the motor.
 * @note   Option A (26-09-2026): the MCSDK does both start-up jobs itself.
 *          1. ALIGNMENT (0.6 A, 700 ms) sets the commutation angle.
 *          2. TC_EncAlignmentCommand() turns the rotor 1 rev in 2 s under
 *             position control; the first Z pulse calls TC_EncoderReset()
 *             (EXTI9_5 ISR) and sets position zero.
 */
static void app_wait_z_homing(void)
{
  TickType_t    z_start    = xTaskGetTickCount();
  TickType_t    z_last_log = z_start;
  AlignStatus_t z_status   = MC_GetAlignmentStatusMotor1();

  /* 2 s search (Z_ALIGNMENT_DURATION). The first bench run showed it still
   * searching at 3 s, so wait up to 6 s real. */
  while ((TC_ALIGNMENT_COMPLETED != z_status) && (TC_ALIGNMENT_ERROR != z_status) &&
         ((xTaskGetTickCount() - z_start) < pdMS_TO_TICKS(6000)))
  {
    vTaskDelay(pdMS_TO_TICKS(5));
    z_status = MC_GetAlignmentStatusMotor1();

    FOC_GuardReport();   /* prints once if the runaway guard tripped */

    if ((xTaskGetTickCount() - z_last_log) >= pdMS_TO_TICKS(100))
    {
      z_last_log = xTaskGetTickCount();
      /* FOC_LogStatus("HOME"); */   /* enable for a 100 ms trace */
    }
  }

  if (TC_ALIGNMENT_COMPLETED == z_status)
  {
    printf("[HOME] Z index found by MCSDK. Position zero set.\r\n");
    app_position_move_test();   /* 100 rad move, measured from the index */
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

/* See app_foc_task.h */
void app_foc_task_run(void const *argument)
{
  TickType_t start_tick;
  uint8_t    app_ok = 1U;   /* 0 after a start-up failure: remaining steps skipped */

  (void)argument;

  /* 1. Start the motor and wait for RUN (MCSDK alignment runs first) */
  MC_StartMotor1();
  start_tick = xTaskGetTickCount();

  while (MCI_GetSTMState(pMCI[M1]) != RUN)
  {
    /* 6 s real: enough for the 2.5 s alignment */
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
  /* 2. Commutation offset calibration (elec_angle.c): measures and corrects
   *    the offset left by the MCSDK alignment, then restarts the MCSDK index
   *    search. */
  if (0U != app_ok)
  {
    if (0U != commutation_calibrate())   /* 0 = offset measured and applied */
    {
      FOC_GuardReport();
      app_stop("Commutation calibration failed");
      app_ok = 0U;
    }
  }
  FOC_GuardReport();

  /* 3 + 4. Z-index homing result, then the position move */
  if (0U != app_ok)
  {
    app_wait_z_homing();
  }
#endif /* APP_DIAG_COMMUTATION */

  /* FIX (26-09-2026): a FreeRTOS task function must never return. The old
   * body in main.c fell off the end, which lands in prvTaskExitError() ->
   * configASSERT -> interrupts disabled + infinite loop. Stay here instead,
   * reporting a late guard trip if one happens. */
  for (;;)
  {
    FOC_GuardReport();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
