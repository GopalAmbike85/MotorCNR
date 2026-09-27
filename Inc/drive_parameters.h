
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM           50000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/*** Encoder **********************/
#define ENC_AVERAGING_FIFO_DEPTH            16 /*!< depth of the FIFO used to average mechanical speed in 0.1Hz resolution */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       50000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           ES_GPIO

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           2 /* 1 !< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT               400             //0
#define PID_TORQUE_KI_DEFAULT               20            /* was 0 (P-only gave ~19% of Iqref). 20/1024 -> ~75 rad/s, below the 5 ms Iqd filter; MCSDK default 2829 is far too fast with that filter */
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 280             //0
#define PID_FLUX_KI_DEFAULT                 20              /* was 0; same reasoning as PID_TORQUE_KI_DEFAULT */
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            256             //16384
#define TF_KIDIV                            1024            //512
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((256))     //LOG2((16384))
#define TF_KIDIV_LOG                        LOG2((1024))        //LOG2((512))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                200/(SPEED_UNIT/10)  //2130/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT                50/(SPEED_UNIT/10)    //328/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

#define POSITION_LOOP_FREQUENCY_HZ          (uint16_t)1000 /*!< Execution rate of position control regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            256
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((256))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             0.8     //1.6

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            0//1440
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

#define PID_POSITION_KP_GAIN                500      /* 500/4096 = 0.12, bench-proven 26-09-2026: stops at the 100 rad target on the FREE rotor. 1000/1024 hunted; 64/4096 + KD 8 hunted. Positive: +Iq drives position +. Retune on the lead screw */
#define PID_POSITION_KI_GAIN                0         //1000
#define PID_POSITION_KD_GAIN                0         /* KD 8/16 hunted on the bench (likely cause: derivative of a 1024-count encoder is too jumpy at 1 kHz) */
#define PID_POSITION_KPDIV                  4096      /* was 1024: finer KP steps */
#define PID_POSITION_KIDIV                  32768
#define PID_POSITION_KDDIV                  16
#define PID_POSITION_KPDIV_LOG              LOG2((4096))
#define PID_POSITION_KIDIV_LOG              LOG2((32768))
#define PID_POSITION_KDDIV_LOG              LOG2((16))
#define PID_POSITION_ANGLE_STEP             10.0
#define PID_POSITION_MOV_DURATION           10.0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              50 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              8 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define M1_ALIGNMENT_DURATION               2500  /*!< ms. Was 1000 (700 before that). MCSDK EAC ramps 0 -> FINAL_I_ALIGNMENT over this whole time and snapshots the encoder at the end with no hold, so a slower ramp lets the light rotor settle (bench: commutation offset varied 5-37 deg, one boot ran away). Self-test (~330 ms) runs at its start */
#define M1_ALIGNMENT_ANGLE_DEG              90 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT_A                 0.6     /*!< A (was 0.04, too weak to align against the lead screw; original 1.6) */
/* With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment */
/* phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt) */
/* being Av the voltage gain between Rshunt and A/D input */

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
