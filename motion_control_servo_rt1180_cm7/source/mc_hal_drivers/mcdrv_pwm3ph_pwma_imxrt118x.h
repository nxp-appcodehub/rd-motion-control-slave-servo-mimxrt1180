/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCDRV_PWM3PH_PWMA_H_
#define _MCDRV_PWM3PH_PWMA_H_

#include "mlib.h"
#include "mlib_types.h"
#include "fsl_device_registers.h"
#include "gmclib.h"
#include "mc_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


typedef struct _mcdrv_pwm3ph_pwma
{
    GMCLIB_3COOR_T_F16 *psDutyABC;    /* pointer to the 3-phase pwm duty cycles */
    PWM_Type *pui32PwmBaseAddress; /* PWMA base address */
    uint16_t ui16PhASubNum;        /* PWMA phase A sub-module number */
    uint16_t ui16PhBSubNum;        /* PWMA phase B sub-module number */
    uint16_t ui16PhCSubNum;        /* PWMA phase C sub-module number */
    uint16_t ui16FaultFixNum;      /* PWMA fault number for fixed over-current fault detection */
    uint16_t ui16FaultAdjNum;      /* PWMA fault number for adjustable over-current fault detection */
    uint16_t ui16FaultAdjNum1;      /* PWMA fault number for adjustable over-current fault detection */
    uint16_t ui16PWMUpdateSel;      /* PWM run in double update mode in one PWM cycle, this variable is used to select updated VAL2/VAL3,which is used to reduce the exetime*/
} mcdrv_pwm3ph_pwma_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Function updates PWM value register
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_LIB
void MCDRV_eFlexPwm3PhDutyUpdate(mcdrv_pwm3ph_pwma_t *this);

/*!
 * @brief Function enables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_LIB
void MCDRV_eFlexPwm3PhOutEnable(mcdrv_pwm3ph_pwma_t *this);

/*!
 * @brief Function disables PWM outputs
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_LIB
void MCDRV_eFlexPwm3PhOutDisable(mcdrv_pwm3ph_pwma_t *this);

/*!
 * @brief Function return actual value of over current flag
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
RAM_FUNC_LIB
bool_t MCDRV_eFlexPwm3PhFaultGet(mcdrv_pwm3ph_pwma_t *this);

/*!
 * @brief Function enables sub-module0~2 counters of an eFlexPWM
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
ALWAYS_INLINE
inline void PWM_SM012_RUN(mcdrv_pwm3ph_pwma_t *this)
{
	this->pui32PwmBaseAddress->MCTRL |= PWM_MCTRL_RUN(0x07);
}

/*!
 * @brief Function enables sub-module0~3 counters of an eFlexPWM
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
ALWAYS_INLINE
inline void PWM_SM0123_RUN(mcdrv_pwm3ph_pwma_t *this)
{
	this->pui32PwmBaseAddress->MCTRL |= PWM_MCTRL_RUN(0x0F);
}



#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_PWM3PH_PWMA_H_ */
