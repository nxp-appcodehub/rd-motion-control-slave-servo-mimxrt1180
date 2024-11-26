/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_


#include "mcdrv_adc_imxrt118x.h"
#include "mcdrv_pwm3ph_pwma_imxrt118x.h"
#include "mcdrv_qdc_imxrt118x.h"
#include "mcdrv_gd3000.h"
#include "mcdrv_sinc_imxrt118x.h"
#include "mc_common.h"
#include "SPC010728Y00.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
  /* data */
  GMCLIB_3COOR_T_F16 sIABCFrac;               /* Measured 3-phase current (FRAC)*/
  frac16_t f16UDcBus;
  frac16_t f16AdcAuxSample;
}adc_results_t;


#define MASK_BIT15 0x8000
#define MASK_BIT14 0x4000
#define MASK_BIT13 0x2000
#define MASK_BIT12 0x1000
#define MASK_BIT11 0x0800
#define MASK_BIT10 0x0400
#define MASK_BIT9  0x0200
#define MASK_BIT8  0x0100
#define MASK_BIT7  0x0080
#define MASK_BIT6  0x0040
#define MASK_BIT5  0x0020
#define MASK_BIT4  0x0010
#define MASK_BIT3  0x0008
#define MASK_BIT2  0x0004
#define MASK_BIT1  0x0002
#define MASK_BIT0  0x0001

#define MOTOR_1	   1
#define MOTOR_2	   2

/******************************************************************************
 * Clock & PWM definition for motor 1
 ******************************************************************************/
#define M1_FAST_LOOP_TS           ((float_t)1.0/(float_t)(M1_FAST_LOOP_FREQ))
#define M1_SLOW_LOOP_TS           ((float_t)1.0/(float_t)(M1_SLOW_LOOP_FREQ))
#define M1_TIME_ONESEC_COUNT      (uint16_t)(M1_FAST_LOOP_FREQ)



/******************************************************************************
 * Clock & PWM definition for motor 2
 ******************************************************************************/
#define M2_FAST_LOOP_TS           ((float_t)1.0/(float_t)(M2_FAST_LOOP_FREQ))
#define M2_SLOW_LOOP_TS           ((float_t)1.0/(float_t)(M2_SLOW_LOOP_FREQ))
#define M2_TIME_ONESEC_COUNT      (uint16_t)(M2_FAST_LOOP_FREQ)

#define M1_FASTLOOP_TIMER_ENABLE() PWM_SM012_RUN(&g_sM1Pwm3ph)
#define M2_FASTLOOP_TIMER_ENABLE() PWM_SM012_RUN(&g_sM2Pwm3ph)
//#define M1_SLOWLOOP_TIMER_ENABLE() TMR1->CHANNEL[0].CTRL |= TMR_CTRL_CM(1)
//#define M2_SLOWLOOP_TIMER_ENABLE() TMR2->CHANNEL[0].CTRL |= TMR_CTRL_CM(1)
#define RESET_TIMER1()             TMR1->CHANNEL[1].CNTR = 0
#define RESET_TIMER2()             TMR1->CHANNEL[2].CNTR = 0
#define RESET_TIMER3()             TMR1->CHANNEL[3].CNTR = 0
#define START_TIMER1()             TMR1->CHANNEL[1].CTRL |= TMR_CTRL_CM(1)
#define START_TIMER2()             TMR1->CHANNEL[2].CTRL |= TMR_CTRL_CM(1)
#define START_TIMER3()             TMR1->CHANNEL[3].CTRL |= TMR_CTRL_CM(1)
#define READ_TIMER1()              TMR1->CHANNEL[1].CNTR
#define READ_TIMER2()              TMR1->CHANNEL[2].CNTR
#define READ_TIMER3()              TMR1->CHANNEL[3].CNTR
//-------------------------------------------------------------------------------
extern void peripherals_manual_init(void);
//extern void SPI_device_select(spi_selection_t eChoice);
extern void adc_etc_init(void);

extern mcdrv_qdc_block_t g_sM1QdcSensor,g_sM2QdcSensor;
extern mcdrv_adc_t g_sM1AdcSensor,g_sM2AdcSensor;
extern mcdrv_pwm3ph_pwma_t g_sM1Pwm3ph,g_sM2Pwm3ph;
extern mcdrv_GD3000_t g_sM1GD3000, g_sM2GD3000;
extern adc_results_t sM1_ADCRslt,sM2_ADCRslt;
extern mcdrv_tamagawa_abs_t g_sM1TamagawaAbs, g_sM2TamagawaAbs;
extern mcdrv_sinc_t g_sM1SincFilter,g_sM2SincFilter;


#ifdef __cplusplus
}
#endif

#endif /* _MC_PERIPH_INIT_H_  */
