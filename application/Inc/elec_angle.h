#ifndef ELEC_ANGLE_H
#define ELEC_ANGLE_H

#include <stdint.h>

extern volatile uint8_t  g_force_angle_en;
extern volatile int16_t  g_force_angle;
extern volatile int16_t  g_openloop_id;
extern volatile int16_t  g_el_raw_dbg;

extern volatile uint8_t  g_id_clear_req;

#define ZS_REQ_IDLE      0U
#define ZS_REQ_PENDING   1U
#define ZS_REQ_DONE      2U
#define ZS_REQ_DROPPED   3U
extern volatile uint8_t  g_zsearch_req;

uint8_t commutation_calibrate(void);

extern volatile uint8_t g_cal_status;

extern volatile int16_t g_cal_offset;

extern volatile uint8_t g_cal_apply;

#endif
