#ifndef APP_FOC_TASK_H
#define APP_FOC_TASK_H

/* ---------------------------------------------------------------------------
 * Application body of the foc_task thread (26-09-2026).
 *
 * CubeMX owns start_foc_control_task() in Src/main.c (it regenerates the
 * function frame). The frame now only calls app_foc_task_run(), so all the
 * start-up / homing / move logic lives here in the application layer.
 * ------------------------------------------------------------------------- */

/**
 * @brief  foc_task body: start motor, commutation calibration, MCSDK Z-index
 *         homing, position move. Never returns (FreeRTOS task rule).
 * @param  argument  task argument from osThreadCreate() (not used).
 */
void app_foc_task_run(void const *argument);

#endif /* APP_FOC_TASK_H */
