/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "pmsm_control.h"
#include "mc_common.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPEED_PI_WITH_DESAT_GAIN	0  // 0: use RTCESL PI. 1: ASR use PI with desaturation gain
#define SPEED_PI_WITH_SATCOMP	0  // 0: use RTCESL PI. 1: ASR use PI with sat_comp
#define CURRENT_PI_WITH_SATCOMP 0  // 0: use RTCESL PI. 1: ACR use PI with sat_comp
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* dead-time compensation voltage table */
static float_t pfltUDtComp[DTCOMP_TABLE_SIZE] = DTCOMP_TABLE_DATA;

/* dead-time compensation look-up table */
static GFLIB_LUT1D_T_FLT sLUTUDtComp;

RAM_FUNC_CRITICAL
static void MCS_DTComp(GMCLIB_2COOR_ALBE_T_FLT *sUAlBeDTComp, 
                GMCLIB_3COOR_T_FLT *sIABC,
                float_t fltUDcBusFilt,
                float_t fltPwrStgCharIRange,
                float_t fltPwrStgCharLinCoeff);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief PMSM field oriented current control.

  This function is used to compute PMSM field oriented current control.

 * @param psFocPMSM     The pointer of the PMSM FOC structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMFocCtrl(mcs_pmsm_foc_t *psFocPMSM)
{

    psFocPMSM->f16PosEl = psFocPMSM->f16PosElExt;    //use the encoder position

    /* 3-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_Clark_FLT(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);

    /************************************************************************************************************/

    /* for open loop control enabled parallel running of FOC
     * open loop electrical position passed to rest of FOC */
    /*
     *    1. In openloop control, DQ current is calculated
     *    2. In closedloop control with encoder (using external position), DQ current is calculated
     *  */
    if (psFocPMSM->bOpenLoop || psFocPMSM->bPosExtOn)
    {
        psFocPMSM->sAnglePosEl.fltSin = GFLIB_Sin_FLTa((acc32_t)psFocPMSM->f16PosEl);
        psFocPMSM->sAnglePosEl.fltCos = GFLIB_Cos_FLTa((acc32_t)psFocPMSM->f16PosEl);
        GMCLIB_Park_FLT(&psFocPMSM->sIAlBe, &psFocPMSM->sAnglePosEl, &psFocPMSM->sIDQ);
    }

    /* perform current control loop if enabled */
    if (psFocPMSM->bCurrentLoopOn)
    {
        /* D current error calculation */
        psFocPMSM->sIDQError.fltD = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltD, psFocPMSM->sIDQ.fltD);

        /* Q current error calculation */
        psFocPMSM->sIDQError.fltQ = MLIB_Sub_FLT(psFocPMSM->sIDQReq.fltQ, psFocPMSM->sIDQ.fltQ);
#if CURRENT_PI_WITH_SATCOMP == 0
        /*** D - controller limitation calculation ***/
        /* Limit the voltage length to the inner circle of a hexagon. 1/(3^-2) = 0.57733*/
        psFocPMSM->sIdPiParams.fltLowerLim = MLIB_MulNeg_FLT((psFocPMSM->fltDutyCycleLimit*0.57733F), psFocPMSM->fltUDcBusFilt);
        psFocPMSM->sIdPiParams.fltUpperLim = MLIB_Mul_FLT((psFocPMSM->fltDutyCycleLimit*0.57733F), psFocPMSM->fltUDcBusFilt);

        /* D current PI controller */
        psFocPMSM->sUDQReq.fltD =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltD, &psFocPMSM->bIdPiStopInteg, &psFocPMSM->sIdPiParams);

        /*** Q - controller limitation calculation ***/
        psFocPMSM->sIqPiParams.fltUpperLim = GFLIB_Sqrt_FLT(
            psFocPMSM->sIdPiParams.fltUpperLim * psFocPMSM->sIdPiParams.fltUpperLim -
            psFocPMSM->sUDQReq.fltD * psFocPMSM->sUDQReq.fltD);
        psFocPMSM->sIqPiParams.fltLowerLim = MLIB_Neg_FLT(psFocPMSM->sIqPiParams.fltUpperLim);
        /* Q current PI controller */
        psFocPMSM->sUDQReq.fltQ =
            GFLIB_CtrlPIpAW_FLT(psFocPMSM->sIDQError.fltQ, &psFocPMSM->bIqPiStopInteg, &psFocPMSM->sIqPiParams);

#else

        sIdPiParamsTest.fltMax = MLIB_Mul_FLT((psFocPMSM->fltDutyCycleLimit*0.57733F), psFocPMSM->fltUDcBusFilt);
        sIdPiParamsTest.fltMin= MLIB_MulNeg_FLT((psFocPMSM->fltDutyCycleLimit*0.57733F), psFocPMSM->fltUDcBusFilt);
        psFocPMSM->sUDQReq.fltD = PI_CTRL_FLT(psFocPMSM->sIDQError.fltD, &sIdPiParamsTest);

        sIdPiParamsTest.fltMax = GFLIB_Sqrt_FLT(
                psFocPMSM->sIdPiParams.fltUpperLim * psFocPMSM->sIdPiParams.fltUpperLim -
                psFocPMSM->sUDQReq.fltD * psFocPMSM->sUDQReq.fltD);
        sIdPiParamsTest.fltMin =  MLIB_Neg_FLT(psFocPMSM->sIqPiParams.fltUpperLim);

        psFocPMSM->sUDQReq.fltQ = PI_CTRL_FLT(psFocPMSM->sIDQError.fltQ, &sIqPiParamsTest);
#endif
    }

    /* 2-phase to 2-phase transformation to stationary ref. frame */
    GMCLIB_ParkInv_FLT(&psFocPMSM->sUDQReq, &psFocPMSM->sAnglePosEl, &psFocPMSM->sUAlBeReq);

    /* dead-time compensation */
    psFocPMSM->sUAlBeDTComp = psFocPMSM->sUAlBeReq;
    if(psFocPMSM->bFlagDTComp)
    {
        MCS_DTComp(&psFocPMSM->sUAlBeDTComp,
                   &psFocPMSM->sIABC,
                   psFocPMSM->fltUDcBusFilt,
                   psFocPMSM->fltPwrStgCharIRange,
                   psFocPMSM->fltPwrStgCharLinCoeff);
    }

    /* DCBus ripple elimination */
    GMCLIB_ElimDcBusRipFOC_F16ff(psFocPMSM->fltUDcBusFilt, &psFocPMSM->sUAlBeDTComp, &psFocPMSM->sUAlBeCompFrac);

    /* space vector modulation */
    psFocPMSM->ui16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeCompFrac, &psFocPMSM->sDutyABC);
}

/*!
 * @brief PMSM field oriented speed control.
 *
   This function is used to compute PMSM field oriented speed control.

 * @param psSpeed       The pointer of the PMSM speed structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMFocCtrlSpeed(mcs_speed_t *psSpeed)
{
    float_t fltTmp;

	/* Speed saturation flag given by the Q current controller saturation flag and speed controller saturation flag */
    psSpeed->bSpeedPiStopInteg = (psSpeed->sSpeedPiParams.bLimFlag | psSpeed->bIqPiLimFlag) &
                                 (bool_t)(MLIB_Abs_FLT(psSpeed->fltSpeedCmd) >= MLIB_Abs_FLT(psSpeed->fltSpeedFilt));
    /* Speed ramp generation */
    psSpeed->fltSpeedRamp = GFLIB_Ramp_FLT(psSpeed->fltSpeedCmd, &psSpeed->sSpeedRampParams);


    /* Controller gain adjustment */
    /*
    if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 10.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.0025F;
    }
    else if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 30.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.014F;
    }
    else if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 50.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.0255F;
    }
    else if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 70.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.037F;
    }
    else if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 90.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.0485F;
    }
    else if(MLIB_Abs_FLT(psSpeed->fltSpeedFilt) < 100.0F*M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF)
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.055F;
    }
    else
    {
    	psSpeed->sSpeedPiParams.fltPGain = 0.06F;
    }
    */

    /* Speed error calculation */
    psSpeed->fltSpeedError = MLIB_Sub_FLT(psSpeed->fltSpeedRamp, psSpeed->fltSpeedFilt);
    psSpeed->fltSpeedErrorDepFlt = MLIB_Sub_FLT(psSpeed->fltSpeedRamp, psSpeed->fltSpeedDeepFilt);
    /* Desired current by the speed PI controller */
#if SPEED_PI_WITH_DESAT_GAIN == 0
#if SPEED_PI_WITH_SATCOMP == 0
    psSpeed->fltIqController =
        GFLIB_CtrlPIpAW_FLT(psSpeed->fltSpeedError, &psSpeed->bSpeedPiStopInteg, &psSpeed->sSpeedPiParams);
#else

    psSpeed->fltIqController = PI_CTRL_FLT(psSpeed->fltSpeedError,psSpeed->fltSpeedErrorDepFlt, &psSpeed->sSpdPiParamsTest);
#endif

#else
    psSpeed->fltIqController =
    		PIControllerDesatUpdate(psSpeed->fltSpeedError, &psSpeed->sSpeedPiParamsDesat);
#endif
    /*   Iq feed forward */
    psSpeed->fltIqFwd = psSpeed->fltIqFwdGain * (psSpeed->fltSpeedRamp - psSpeed->fltSpeedRamp_1);
    psSpeed->fltIqFwdFilt = GDFLIB_FilterIIR1_FLT(psSpeed->fltIqFwd, &psSpeed->sIqFwdFilter);
    fltTmp = psSpeed->fltIqController + psSpeed->fltIqFwdFilt;

#if SPEED_PI_WITH_DESAT_GAIN == 1
    if(fltTmp > psSpeed->sSpeedPiParamsDesat.sCoeff.fltUpperLim)
    {
    	psSpeed->fltIqReq = psSpeed->sSpeedPiParamsDesat.sCoeff.fltUpperLim;
    }
    else if(fltTmp < psSpeed->sSpeedPiParamsDesat.sCoeff.fltUpperLim)
    {
    	psSpeed->fltIqReq = psSpeed->sSpeedPiParamsDesat.sCoeff.fltLowerLim;
    }
    else
    {
    	psSpeed->fltIqReq = fltTmp;
    }
#else

    if(fltTmp > psSpeed->sSpeedPiParams.fltUpperLim)
    {
    	psSpeed->fltIqReq = psSpeed->sSpeedPiParams.fltUpperLim;
    }
    else if(fltTmp < psSpeed->sSpeedPiParams.fltLowerLim)
    {
    	psSpeed->fltIqReq = psSpeed->sSpeedPiParams.fltLowerLim;
    }
    else
    {
    	psSpeed->fltIqReq = fltTmp;
    }
#endif
    psSpeed->fltSpeedRamp_1 = psSpeed->fltSpeedRamp;
}

/*!
 * @brief An 2nd order filter is used to realize trajectory. This function initializes the filter.

 * @param this       The pointer of the filter
 *
 * @return None
 */
RAM_FUNC_CRITICAL void trajectoryFilterInit(trajectory_filter_t *this)
{

	this->i32In = 0;
	this->i64Out = 0;
	this->i64Out_1 = 0;
	this->i64M = 0;
	this->i64M_1 = 0;

}

/*!
 * @brief An 2nd order filter is used to realize trajectory. This function does the filtering calculation.
          The input can be any Qm.n format, where m+n=32

 * @param this       The pointer of the filter
 *
 * @return result of the filter
 */
RAM_FUNC_CRITICAL int32_t trajectoryFilterUpdate(trajectory_filter_t *this)
{

	int64_t i64Tmp;
	uint32_t ui32Tmp1;
	this->i64A = (((int64_t)this->i32In)<<32) - 2*this->i64M_1 - this->i64Out_1; /* Q16.48 */
	ui32Tmp1 = (uint32_t)this->f32W;
	/*
	 * Determine the positive or negative of A, if it is a negative number will be inverted,
	 * to ensure that the last 63bit is the original code of the value.Store the sign of A
	*/
	if(this->i64A<0)
	{
		i64Tmp = -this->i64A;
		this->i32ASign = -1;
	}
	else{
		i64Tmp = this->i64A;
		this->i32ASign = 1;
	}
	/*
	 * Splitting the A into two uint32-type numbers for processing guarantees the highest precision.
	*/
	this->ui32Ah = (uint32_t)(i64Tmp >>32 );
	this->ui32Al = (uint32_t)i64Tmp;
	this->ui64B = ((uint64_t) this->ui32Ah *ui32Tmp1) <<1;
	this->ui64C = ((uint64_t)this->ui32Al* ui32Tmp1)>>31;
	this->i64M = this->i64M_1 + ((int64_t)(this->ui64B +this->ui64C))*this->i32ASign;
	if(this->i64M<0)
	{
		i64Tmp = -this->i64M;
		this->i32MSign = -1;
	}
	else
	{
		i64Tmp = this->i64M;
		this->i32MSign = 1;
	}
	this->ui32Mh = (uint32_t)(i64Tmp >>32 );
	this->ui32Ml = (uint32_t)i64Tmp;
	this->ui64D = ((uint64_t)this->ui32Mh *ui32Tmp1) <<1;
	this->ui64E =  ((uint64_t)this->ui32Ml* ui32Tmp1)>>31;
	this->i64Out = this->i64Out_1 + ((int64_t)(this->ui64D +this->ui64E)) *this->i32MSign;
	this->i64M_1 = this->i64M;
	this->i64Out_1 = this->i64Out;

	return (int32_t)(this->i64Out>>32);
}

/*!
 * @brief PMSM field oriented position control.
 *
   This function is used to compute PMSM field oriented position control.

 * @param psPosition       The pointer of the PMSM position structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMFocCtrlPosition(mcs_position_t *psPosition)
{
	int32_t i32Tmp;
	int64_t i64Tmp;

	if(psPosition->bIsRandomPosition == FALSE)
	{
		psPosition->sCurveRef.i32Q16PosCmd = psPosition->i32Q16PosCmd  - psPosition->i32Q16PosOffSet;

		/* Trajectory reference creation */
		psPosition->sCurveRef.i32Q16PosRamp = GFLIB_Ramp_F32(psPosition->sCurveRef.i32Q16PosCmd, &psPosition->sCurveRef.sPosRamp);
		psPosition->sCurveRef.sTrajFilter.i32In = psPosition->sCurveRef.i32Q16PosRamp;
		psPosition->sCurveRef.i32Q16PosFilt = trajectoryFilterUpdate(&psPosition->sCurveRef.sTrajFilter);
		psPosition->sCurveRef.i32Q16PosErr = psPosition->sCurveRef.i32Q16PosRamp - psPosition->sCurveRef.i32Q16PosRamp_1;

		psPosition->sCurveRef.i32Q16PosRamp_1 = psPosition->sCurveRef.i32Q16PosRamp ;
		/* Position loop */
		psPosition->i32Q16PosRef = psPosition->sCurveRef.i32Q16PosFilt;
	}
	else
	{
		psPosition->i32Q16PosRef = psPosition->i32Q16PosCmd;
	}
	psPosition->i32Q16PosErr = psPosition->i32Q16PosRef - psPosition->i32Q16PosFdbk;
	i32Tmp = MLIB_MulSat_A32(psPosition->i32Q16PosErr, psPosition->a32PosGain)>>1; /* Q16.16 * Q17.15 = Q33.31, removing the last 16bits and the highest 16bits, we get Q17.15 format */
	if(i32Tmp >= ACC32(1.0))
	{
		psPosition->f16SpeedController = psPosition->f16SpeedRefLim;
	}
	else if(i32Tmp <= ACC32(-1.0))
	{
		psPosition->f16SpeedController = MLIB_Neg_F16(psPosition->f16SpeedRefLim);
	}
	else
	{
		psPosition->f16SpeedController = MLIB_Mul_F16((int16_t)i32Tmp, psPosition->f16SpeedRefLim);
	}
	psPosition->fltSpeedController = MLIB_ConvSc_FLTsf(psPosition->f16SpeedController, psPosition->fltFracToAngularSpeedCoeff);

	/* Speed reference feed forward */
	psPosition->i32PosrefErr = psPosition->i32Q16PosRef - psPosition->i32Q16PosRef_1; /* Get revolution deviation */
	i64Tmp = (int64_t)psPosition->i32PosrefErr * psPosition->i32PosLoopFreq; /* Q16.16 * Q32.0 = Q48.16, get the derivative of position reference against time: i64Tmp represents mechanical frequency here */
	i32Tmp = i64Tmp>>1; /* Q48.16 -> Q17.15 */
	psPosition->fltSpeedFwdNoGain = MLIB_ConvSc_FLTaf(i32Tmp, psPosition->fltFreqToAngularSpeedCoeff);
	psPosition->fltSpeedFwdNoGainFilter = GDFLIB_FilterIIR1_FLT(psPosition->fltSpeedFwdNoGain, &psPosition->sSpdFwdFilter);
	psPosition->fltSpeedFwd = psPosition->fltSpeedFwdNoGainFilter * psPosition->fltGainSpeedFwd;

	psPosition->fltSpeedRef = psPosition->fltSpeedFwd + psPosition->fltSpeedController;
    if(psPosition->fltSpeedRef > (psPosition->fltSpeedRefLim * psPosition->fltRpmToAngularSpeedCoeff))
    {
    	psPosition->fltSpeedRef = (psPosition->fltSpeedRefLim * psPosition->fltRpmToAngularSpeedCoeff);
    }
    else if(psPosition->fltSpeedRef < -(psPosition->fltSpeedRefLim * psPosition->fltRpmToAngularSpeedCoeff))
    {
    	psPosition->fltSpeedRef = -(psPosition->fltSpeedRefLim * psPosition->fltRpmToAngularSpeedCoeff);
    }

    /* Store historical data */
	psPosition->i32Q16PosRef_1 = psPosition->i32Q16PosRef;
}

/*!
 * @brief PMSM 2-step rotor alignment - 120deg in first step and 0deg in second.
 *
 * This function is used for alignment rotor in two steps - 120deg in first step and 0deg in second
 *
 * @param psAlignment   The pointer of the motor control alignment structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMAlignment(mcs_alignment_t *psAlignment)
{
    /* first half duration time is position set to 120 degree */
    if (psAlignment->ui16TimeHalf > 0)
    {
        psAlignment->f16PosAlign = FRAC16(120.0 / 180.0);
        psAlignment->ui16TimeHalf--;
    }
    else
    {
        psAlignment->f16PosAlign = FRAC16(0.0);
    }

}

/*!
 * @brief PMSM Open Loop Start-up
 *
 * This function is used to PMSM Open Loop Start-up
 *
 * @param psStartUp     The pointer of the PMSM open loop start up parameters structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMOpenLoopStartUp(mcs_pmsm_startup_t *psStartUp)
{
    /* Open loop startup speed ramp */
    psStartUp->fltSpeedRampOpenLoop = GFLIB_Ramp_FLT(psStartUp->fltSpeedReq, 
                                                     &psStartUp->sSpeedRampOpenLoopParams);

    /* generation of open loop position from the required speed */
    psStartUp->f16PosGen = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(psStartUp->fltSpeedRampOpenLoop,psStartUp->fltSpeedMax), 
                                                &psStartUp->sSpeedIntegrator);

    /* position merging starts above merging speed threshold*/
    if(psStartUp->bMergeFlag == TRUE)
    {
        if (MLIB_Abs_FLT(psStartUp->fltSpeedRampOpenLoop) >= psStartUp->fltSpeedCatchUp)
        {
            /* increment position merging coefficient */
            psStartUp->f16RatioMerging = MLIB_AddSat_F16(psStartUp->f16RatioMerging, psStartUp->f16CoeffMerging);

            /* merging equation */
            psStartUp->f16PosMerged = MLIB_Add_F16(
                psStartUp->f16PosGen,
                MLIB_Mul_F16(MLIB_Sub_F16(psStartUp->f16PosEst, psStartUp->f16PosGen), psStartUp->f16RatioMerging));
        }
        else
        { 
            psStartUp->f16PosMerged = psStartUp->f16PosGen;
        }
    }
    else
    {
    	psStartUp->f16PosMerged = psStartUp->f16PosGen;
    }

    /* clear open loop flag */
    if (psStartUp->f16RatioMerging == FRAC16(1.0))
        psStartUp->bOpenLoop = FALSE;
}

/*!
 * @brief PMSM scalar control, voltage is set based on required speed
 *
 * This function is used for alignment rotor in two steps - 120deg in first step and 0deg in second
 *
 * @param psScalarPMSM   The pointer of the PMSM scalar control structure
 *
 * @return None
 */
RAM_FUNC_CRITICAL void MCS_PMSMScalarCtrl(mcs_pmsm_scalar_ctrl_t *psScalarPMSM)
{
    /* this part of code is executed when scalar control is turned-on */
    /* frequency ramp */
    psScalarPMSM->fltFreqRamp =
        GFLIB_Ramp_FLT(psScalarPMSM->fltFreqCmd, &psScalarPMSM->sFreqRampParams);

    /* voltage calculation */
    psScalarPMSM->sUDQReq.fltQ = psScalarPMSM->fltVHzGain * psScalarPMSM->fltFreqRamp;
    psScalarPMSM->sUDQReq.fltD = 0.0F;

    /* stator voltage angle , used the same integrator as for the open-loop start up*/
    psScalarPMSM->f16PosElScalar = GFLIB_Integrator_F16(MLIB_ConvSc_F16ff(psScalarPMSM->fltFreqRamp, psScalarPMSM->fltFreqMax ),
                                                        &psScalarPMSM->sFreqIntegrator);    
}

/******************************************************************************
@brief   Dead-time compensation using LUT wit interpolation

@param   N/A

@return  N/A
******************************************************************************/
RAM_FUNC_CRITICAL static void MCS_DTComp(GMCLIB_2COOR_ALBE_T_FLT *sUAlBeDTComp,
                GMCLIB_3COOR_T_FLT *sIABC,
                float_t fltUDcBusFilt,
                float_t fltPwrStgCharIRange,
                float_t fltPwrStgCharLinCoeff)
{
    register GMCLIB_3COOR_T_FLT sUABCErr;
    register float_t            fltUerrMax;
    register int16_t            i16CurrSign;

    /* maximal error voltage */
    fltUerrMax = *pfltUDtComp;

    /* compensate phase A */
    i16CurrSign = (sIABC->fltA > fltPwrStgCharIRange) - 
                  (sIABC->fltA < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltA = GFLIB_Lut1D_FLT(sIABC->fltA, 
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltA = i16CurrSign * 
            ((MLIB_Abs_FLT(sIABC->fltA) - fltPwrStgCharIRange) * 
             fltPwrStgCharLinCoeff - fltUerrMax);

    /* compensate phase B */
    i16CurrSign = (sIABC->fltB > fltPwrStgCharIRange) - 
                  (sIABC->fltB < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltB = GFLIB_Lut1D_FLT(sIABC->fltB,
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltB = i16CurrSign * 
          ((MLIB_Abs_FLT(sIABC->fltB) - fltPwrStgCharIRange) * 
           fltPwrStgCharLinCoeff - fltUerrMax);

    /* compensate phase C */
    i16CurrSign = (sIABC->fltC > fltPwrStgCharIRange) - 
                  (sIABC->fltC < -fltPwrStgCharIRange);
    if (!i16CurrSign)
        sUABCErr.fltC = GFLIB_Lut1D_FLT(sIABC->fltC, 
                                           pfltUDtComp, 
                                           &sLUTUDtComp);
    else
        sUABCErr.fltC = i16CurrSign * 
          ((MLIB_Abs_FLT(sIABC->fltC) - fltPwrStgCharIRange) * 
           fltPwrStgCharLinCoeff - fltUerrMax);

    /* add compensation voltages */
    sUAlBeDTComp->fltAlpha += (0.333333333333F) * fltUDcBusFilt * 
                               (sUABCErr.fltA + sUABCErr.fltA - 
                                sUABCErr.fltB - sUABCErr.fltC);

    sUAlBeDTComp->fltBeta += (0.5773502691896F) * fltUDcBusFilt * 
                              (sUABCErr.fltB - sUABCErr.fltC);
}

/*!
 * @brief    An 8th order Band Stop Filter, which is realized by 4 Second Order Series(SOS).
            This function updates the filter.

 * @param  ptr   The pointer of the filter structure
 * @param  fltIn  Input data
 * @return Filter result
 */
RAM_FUNC_CRITICAL float_t BSF_update(FILTER_T *ptr, float_t fltIn)
{
	uint16_t ui16Index;
	float_t fltTmp;

	/* 1st IIR2 */
	ptr->sSOS_array[0].fltIn = ptr->sSOS_array[0].sCoeff.fltScale * fltIn;
	ptr->sSOS_array[0].fltW0 = ptr->sSOS_array[0].fltIn - ptr->sSOS_array[0].sCoeff.fltA1 * ptr->sSOS_array[0].fltW1 - ptr->sSOS_array[0].sCoeff.fltA2 * ptr->sSOS_array[0].fltW2; /* input*scale + a1*w1 + a2*w2 */
	ptr->sSOS_array[0].fltOut = ptr->sSOS_array[0].sCoeff.fltB0 * ptr->sSOS_array[0].fltW0 + ptr->sSOS_array[0].sCoeff.fltB1 * ptr->sSOS_array[0].fltW1 + ptr->sSOS_array[0].sCoeff.fltB2 * ptr->sSOS_array[0].fltW2; /* b0*w0 + b1*w1 + b2*w2 */
	ptr->sSOS_array[0].fltW2 = ptr->sSOS_array[0].fltW1;
	ptr->sSOS_array[0].fltW1 = ptr->sSOS_array[0].fltW0; /* update states */

	for(ui16Index = 0; ui16Index <=2; ui16Index++ )
	{
		ptr->sSOS_array[ui16Index+1].fltIn = ptr->sSOS_array[ui16Index+1].sCoeff.fltScale * ptr->sSOS_array[ui16Index].fltOut;
		ptr->sSOS_array[ui16Index+1].fltW0 = ptr->sSOS_array[ui16Index+1].fltIn - ptr->sSOS_array[ui16Index+1].sCoeff.fltA1 * ptr->sSOS_array[ui16Index+1].fltW1 - ptr->sSOS_array[ui16Index+1].sCoeff.fltA2 * ptr->sSOS_array[ui16Index+1].fltW2; /* input*scale + a1*w1 + a2*w2 */
		ptr->sSOS_array[ui16Index+1].fltOut = ptr->sSOS_array[ui16Index+1].sCoeff.fltB0 * ptr->sSOS_array[ui16Index+1].fltW0 + ptr->sSOS_array[ui16Index+1].sCoeff.fltB1 * ptr->sSOS_array[ui16Index+1].fltW1 + ptr->sSOS_array[ui16Index+1].sCoeff.fltB2 * ptr->sSOS_array[ui16Index+1].fltW2; /* b0*w0 + b1*w1 + b2*w2 */
		ptr->sSOS_array[ui16Index+1].fltW2 = ptr->sSOS_array[ui16Index+1].fltW1;
		ptr->sSOS_array[ui16Index+1].fltW1 = ptr->sSOS_array[ui16Index+1].fltW0; /* update states */
	}

	fltTmp = ptr->sSOS_array[3].fltOut * ptr->fltOutScale;

	return fltTmp;
}

/*!
 * @brief    An 8th order Band Stop Filter, which is realized by 4 Second Order Series(SOS).
            This function initializes the internal states of the filter.

 * @param  fltIn  Input data
 * @return non
 */
void BSF_init(FILTER_T *ptr)
{
	uint16_t ui16Index;

	for(ui16Index = 0; ui16Index <= 4; ui16Index++)
	{
		ptr->sSOS_array[ui16Index].fltW0 = 0;
		ptr->sSOS_array[ui16Index].fltW1 = 0;
		ptr->sSOS_array[ui16Index].fltW2 = 0;
	}
}

/*!
 * @brief    A PI controller with desaturation gain control. This function performs one step calculation of the controller.

 * @param  fltErr Input to the controller
 * @param  ptr    Pointer to the controller structure
 * @return Controller output
 */
RAM_FUNC_CRITICAL float_t PIControllerDesatUpdate(float_t fltErr, pi_controller_desat_t *ptr)
{
	float_t fltTmp;

	ptr->fltErr = fltErr;

	ptr->fltProp = ptr->sCoeff.fltPGain * ptr->fltErr;
	ptr->fltIntegral = ptr->fltIntegral_1 + ptr->sCoeff.fltIGain * ptr->fltErr;
	fltTmp = ptr->fltOut_1 - ptr->fltPresatOut_1;
	ptr->fltDesat = fltTmp * ptr->sCoeff.fltDesatGain;
	ptr->fltPresatOut = ptr->fltProp + ptr->fltIntegral + ptr->fltDesat;

	if(ptr->fltPresatOut > ptr->sCoeff.fltUpperLim)
	{
		ptr->fltOut = ptr->sCoeff.fltUpperLim;
	}
	else if(ptr->fltPresatOut < ptr->sCoeff.fltLowerLim)
	{
		ptr->fltOut = ptr->sCoeff.fltLowerLim;
	}
	else
	{
		ptr->fltOut = ptr->fltPresatOut;
	}

	ptr->fltIntegral_1 = ptr->fltIntegral;
	ptr->fltPresatOut_1 = ptr->fltPresatOut;
	ptr->fltOut_1 = ptr->fltOut;

	return ptr->fltOut;

}
/*!
 * @brief    A PI controller with desaturation gain control-Vertion 2. This function performs one step calculation of the controller.

 * @param  fltErr Input to the controller
 * @param  ptr    Pointer to the controller structure
 * @return Controller output
 */
RAM_FUNC_CRITICAL float PI_CTRL_FLT(float_t fltIn, float_t fltFliterIn,PI_CTRL_FLT_T *ptr)
{
	float fltTmp_P, fltTmp_I, fltTmp;
	float fltOut;
	// Proportional part
	fltTmp_P = ptr->fltPGain * fltFliterIn;
	// Integral part
	fltTmp_I = ptr->fltIGain * fltIn + ptr->fltSat;
	fltTmp = ptr->fltInteg_1;
	fltTmp += fltTmp_I;
	ptr->fltInteg_1 = fltTmp;
	// Output limitation
	fltTmp += fltTmp_P;
	if(fltTmp > ptr->fltMax)
	{
		fltOut = ptr->fltMax;
	}
	else if(fltTmp < ptr->fltMin)
	{
		fltOut = ptr->fltMin;
	}
	else
	{
		fltOut = fltTmp;
	}
	// Update saturation portion
	ptr->fltSat = fltOut - fltTmp;
	return fltOut;
}


/*!
 * @brief    A PI controller with desaturation gain control. This function initializes the internal states of the controller.

 * @param  ptr    Pointer to the controller structure
 * @return none
 */
RAM_FUNC_CRITICAL void PIControllerDesatInit(pi_controller_desat_t *ptr)
{
	ptr->fltIntegral = 0;
	ptr->fltIntegral_1 = 0;
	ptr->fltOut = 0;
	ptr->fltOut_1 = 0;
	ptr->fltPresatOut = 0;
	ptr->fltPresatOut_1 = 0;
}
/*!
 * @brief    A PI controller with desaturation gain control_Vertion 2. This function initializes the internal states of the controller.

 * @param  ptr    Pointer to the controller structure
 * @return none
 */
void PIController_Init(PI_CTRL_FLT_T *ptr)
{
	ptr->fltSat = 0;
	ptr->fltInteg_1 = 0;
}
