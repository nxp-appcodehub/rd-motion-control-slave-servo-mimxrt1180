/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
#ifndef M2_SM_REF_SOL_H
#define M2_SM_REF_SOL_H

#include "sm_ref_sol_comm.h"
#include "m2_pmsm_appconfig.h"
#include "state_machine.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

   
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern bool_t g_bM2SwitchAppOnOff;
extern mcdef_pmsm_t g_sM2Drive;
extern sm_app_ctrl_t g_sM2Ctrl;
extern run_substate_t g_eM2StateRun;

extern volatile float g_fltM2voltageScale;
extern volatile float g_fltM2DCBvoltageScale;
extern volatile float g_fltM2currentScale;
extern volatile float g_fltM2speedScale;
extern volatile float g_fltM2speedAngularScale;
extern void M2_FastLoopCriticalCode(void);
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * API
 ******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* M2_SM_REF_SOL_H */

