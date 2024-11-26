/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef M1_SM_REF_SOL_H
#define M1_SM_REF_SOL_H

#include "sm_ref_sol_comm.h"
#include "m1_pmsm_appconfig.h"
#include "state_machine.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


   
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern bool_t g_bM1SwitchAppOnOff,g_bM1SwitchAppOnOff_1;
extern mcdef_pmsm_t g_sM1Drive;
extern sm_app_ctrl_t g_sM1Ctrl;
extern run_substate_t g_eM1StateRun;

extern volatile float g_fltM1voltageScale;
extern volatile float g_fltM1DCBvoltageScale;
extern volatile float g_fltM1currentScale;
extern volatile float g_fltM1speedScale;
extern volatile float g_fltM1speedAngularScale;
extern void M1_FastLoopCriticalCode(void);
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* M1_SM_REF_SOL_H */

