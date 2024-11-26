/*
    * Copyright 2022 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef MC_COMMON_H
#define MC_COMMON_H

#include "m1_pmsm_appconfig.h"
#include "m2_pmsm_appconfig.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENABLED  1
#define DISABLED 0
#define TAMAGAWA_ABS_ENC  0
#define OPTICAL_ENC       1
#define RUN_DEMO_SELECTION   1 //0 is demo1 ; 1 is demo2

#define SINC_CLOSEDLOOP   ENABLED /* ENABLED: Use currents/voltage from sincFilter for fast loop; DISABLED: Use currents/voltage from internal ADC for fast loop */
#define POSITION_SENSOR   TAMAGAWA_ABS_ENC /* TAMAGAWA_ABS_ENC or OPTICAL_ENC */
#define USEIIRFORBW  ENABLED /* ENABLED: Use IIR for speed calculate; DISABLE: Use IIR for current request */
#define FEATURE_MC_INVERTER_OUTPUT_DEBUG_ENABLE 1
#define FEATURE_MC_INVERTER_OUTPUT_ENABLE 0
#define EtherCAT_LOOP_CYCLE 8000
#define AVERAGE_CNT M1_SLOW_LOOP_FREQ/EtherCAT_LOOP_CYCLE
#ifndef RAM_FUNC_CRITICAL
#if 0
#define RAM_FUNC_CRITICAL __attribute__((section(".ramfunc.$SRAM_ITC_cm7")))
#else
#define RAM_FUNC_CRITICAL
#endif
#endif

#if 1
#define ALWAYS_INLINE __attribute__((always_inline))
#else
#define ALWAYS_INLINE _Pragma("inline=forced")
#endif

/* CPU load measurement SysTick START / STOP macros */
#define SYSTICK_START_COUNT() (SysTick->VAL = SysTick->LOAD)
#define SYSTICK_STOP_COUNT(par1)   \
    par1          = SysTick->LOAD - SysTick->VAL

#endif /* MC_COMMON_H */
