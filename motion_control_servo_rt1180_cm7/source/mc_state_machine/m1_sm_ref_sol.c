/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "m1_sm_ref_sol.h"
#include "mid_sm_states.h"
#include "api_motorcontrol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define M1_SVM_SECTOR_DEFAULT (2) /* default SVM sector */

#define FREQ_CUR_SCALE_M1 8000.0 /* [Hz], frequency scale for current command in bandwidth test */
#define FREQ_SCALE_M1     1200.0  /* [Hz], frequency scale for speed command in bandwidth test */
#define FREQ_POS_SCALE_M1 250.0  /* [Hz], frequency scale for position command in bandwidth test */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* State and transition functions for the main state machine */
RAM_FUNC_CRITICAL 
static void M1_StateFaultFast(void);
RAM_FUNC_CRITICAL
static void M1_StateInitFast(void);
RAM_FUNC_CRITICAL
static void M1_StateStopFast(void);
RAM_FUNC_CRITICAL 
static void M1_StateRunFast(void);

RAM_FUNC_CRITICAL
static void M1_StateFaultSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateInitSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateStopSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunSlow(void);

RAM_FUNC_CRITICAL  
static void M1_TransFaultStop(void);
RAM_FUNC_CRITICAL  
static void M1_TransInitFault(void);
RAM_FUNC_CRITICAL  
static void M1_TransInitStop(void);
RAM_FUNC_CRITICAL  
static void M1_TransStopFault(void);
RAM_FUNC_CRITICAL  
static void M1_TransStopRun(void);
RAM_FUNC_CRITICAL  
static void M1_TransRunFault(void);
RAM_FUNC_CRITICAL  
static void M1_TransRunStop(void);

/* State and transition functions for the sub-state machine in Run state of main state machine */
RAM_FUNC_CRITICAL   
static void M1_StateRunCalibFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunMeasureFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunReadyFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunAlignFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunStartupFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunSpinFast(void);
RAM_FUNC_CRITICAL   
static void M1_StateRunFreewheelFast(void);

RAM_FUNC_CRITICAL
static void M1_StateRunCalibSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunMeasureSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunReadySlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunAlignSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunStartupSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunSpinSlow(void);
RAM_FUNC_CRITICAL
static void M1_StateRunFreewheelSlow(void);

RAM_FUNC_CRITICAL
static void M1_TransRunCalibReady(void);
RAM_FUNC_CRITICAL
static void M1_TransRunCalibMeasure(void);
RAM_FUNC_CRITICAL 
static void M1_TransRunMeasureReady(void);
RAM_FUNC_CRITICAL 
static void M1_TransRunReadyAlign(void);
RAM_FUNC_CRITICAL
static void M1_TransRunAlignStartup(void);
RAM_FUNC_CRITICAL 
static void M1_TransRunAlignReady(void);
RAM_FUNC_CRITICAL
static void M1_TransRunAlignSpin(void);
RAM_FUNC_CRITICAL
static void M1_TransRunStartupSpin(void);
RAM_FUNC_CRITICAL 
static void M1_TransRunStartupFreewheel(void);
RAM_FUNC_CRITICAL 
static void M1_TransRunSpinFreewheel(void);
RAM_FUNC_CRITICAL
static void M1_TransRunFreewheelReady(void);
RAM_FUNC_CRITICAL
static void M1_TransRunReadySpin(void);

RAM_FUNC_CRITICAL 
static void M1_ClearFOCVariables(void);

RAM_FUNC_CRITICAL
static void M1_FaultDetection(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Main control structure */
mcdef_pmsm_t g_sM1Drive;

/*! @brief Main application switch */
bool_t g_bM1SwitchAppOnOff;

/*! @brief M1 structure */
run_substate_t g_eM1StateRun;

/*! @brief PI controller structure */
PI_CTRL_FLT_T sIqPiParamsTest;
PI_CTRL_FLT_T sIdPiParamsTest;
PI_CTRL_FLT_T sPosPiParamsTest;

/*! @brief FreeMASTER scales */
/*! DO NOT USE THEM in the code to avoid float library include */
volatile float g_fltM1voltageScale;
volatile float g_fltM1DCBvoltageScale;
volatile float g_fltM1currentScale;
volatile float g_fltM1speedScale;
volatile float g_fltM1speedAngularScale;

volatile static uint16_t ui16PWMOutputDebugFlagM1;

/*! @brief Application state machine table - fast */
static sm_app_state_fcn_t s_M1_STATE_FAST = {M1_StateFaultFast, M1_StateInitFast, M1_StateStopFast, M1_StateRunFast};

/*! @brief Application state machine table - slow */
static sm_app_state_fcn_t s_M1_STATE_SLOW = {M1_StateFaultSlow, M1_StateInitSlow, M1_StateStopSlow, M1_StateRunSlow};

/*! @brief Application sub-state function field - fast */
volatile static pfcn_void_void s_M1_STATE_RUN_TABLE_FAST[7] = {M1_StateRunCalibFast, M1_StateRunReadyFast,
                                                            M1_StateRunAlignFast, M1_StateRunStartupFast,
                                                            M1_StateRunSpinFast,  M1_StateRunFreewheelFast,
                                                            M1_StateRunMeasureFast};

/*! @brief Application sub-state function field - slow */
volatile static pfcn_void_void s_M1_STATE_RUN_TABLE_SLOW[7] = {M1_StateRunCalibSlow, M1_StateRunReadySlow,
                                                            M1_StateRunAlignSlow, M1_StateRunStartupSlow,
                                                            M1_StateRunSpinSlow,  M1_StateRunFreewheelSlow,
                                                            M1_StateRunMeasureSlow};

/*! @brief Application state-transition functions field  */
static  sm_app_trans_fcn_t s_TRANS = {M1_TransFaultStop, M1_TransInitFault, M1_TransInitStop, M1_TransStopFault,
                                           M1_TransStopRun,   M1_TransRunFault,  M1_TransRunStop};

/*! @brief  State machine structure declaration and initialization */
sm_app_ctrl_t g_sM1Ctrl = {
    /* g_sM1Ctrl.psState, User state functions  */
    &s_M1_STATE_FAST,

    /* g_sM1Ctrl.psState, User state functions  */
    &s_M1_STATE_SLOW,

    /* g_sM1Ctrl..psTrans, User state-transition functions */
    &s_TRANS,

    /* g_sM1Ctrl.uiCtrl, Default no control command */
    SM_CTRL_NONE,

    /* g_sM1Ctrl.eState, Default state after reset */
    kSM_AppInit};

/******** For speed loop bandwidth test ********/
volatile static uint16_t ui16SinSpeedCmdSwitchM1 = 0; /* A switch to turn on/off speed loop bandwidth test in speed FOC mode */
volatile static frac32_t f32AngleM1;
volatile static frac32_t f32FreqInM1;  /* Frequency of sinusoidal speed command */
volatile static float_t  fltSinM1;     /* Sin() of speed command frequency */
volatile static float_t  fltSpeedCmdAmplitudeM1 = 2*PI*25.0F; /* Amplitude of sinusoidal speed command */
volatile static float_t  fltSpeedoffset ;
float_t  fltSpeedCmdTestM1; /* Sinusoidal speed command */

/***********************************************/

/******** For current loop bandwidth test ********/
volatile static uint16_t ui16SinCurrentCmdSwitchM1 = 0;  /* A switch to turn on/off current loop bandwidth test in current FOC mode */
volatile static frac32_t f32AngleCurM1;
volatile static frac32_t f32FreqInCurM1;  /* Frequency of sinusoidal current command */
volatile static float_t  fltSinCurM1;     /* Sin() of current command frequency */
volatile static float_t  fltCurrentCmdAmplitudeM1 = 2.0F; /* Amplitude of sinusoidal current command */
volatile static float_t  fltCurrentCmdTestM1; /* Sinusoidal current command */

/***********************************************/

/******** For position loop bandwidth test ********/
volatile static uint16_t ui16SinPosCmdSwitchM1 = 0; /* A switch to turn on/off position loop bandwidth test in current FOC mode */
volatile static frac32_t f32AnglePosM1;
volatile static frac32_t f32FreqInPosM1;  /* Frequency of sinusoidal position command */
volatile static frac16_t f16SinPosM1;     /* Sin() of position command frequency */
volatile static int32_t  i32Q16PosCmdAmplitudeM1 = (int32_t)(0.5*65536); /* Amplitude of sinusoidal position command */
volatile static int32_t  i32Q16PosCmdTestM1; /* Sinusoidal position command */

extern bool_t bPosInitAlginFlag;

/*structure for postion timestamp capture*/
//static mcdrv_position_compensate_t g_sM1PositionCompensate;
//int16_t i16POSTest=1 ;
/***********************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Fault state called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateFaultFast(void)
{
	MCDRV_Curr3Ph2ShGet(&g_sM1AdcSensor);
	MCDRV_VoltDcBusGet(&g_sM1AdcSensor);
	MCDRV_AuxValGet(&g_sM1AdcSensor);

    /* get rotor position and speed from quadrature encoder sensor */
#if POSITION_SENSOR == OPTICAL_ENC    
	if(g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)
	{
		MCDRV_GetRotorCurrentPos(&g_sM1QdcSensor);
		MCDRV_GetRotorCurrentRev(&g_sM1QdcSensor);
		g_sM1Drive.f16PosElEnc = g_sM1QdcSensor.f16PosElec;
	}
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    if(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE)
	{
//		g_sM1PositionCompensate.f16PosElec = g_sM1TamagawaAbs.f16PosElec;       //This function is used for encoder angle compensation
//		g_sM1PositionCompensate.fltSpeed = g_sM1Drive.sSpeed.fltSpeedFilt;
//		MCDRV_PositionCompensateCurSampleTrigPosCal(&g_sM1PositionCompensate);
//		if(i16POSTest == 0 )
//		{
	        g_sM1Drive.f16PosElEnc = g_sM1TamagawaAbs.f16PosElec;
//		}
//		else if(i16POSTest ==1)
//		{
//	        g_sM1Drive.f16PosElEnc = g_sM1PositionCompensate.f16CompPosElec;
//		}
    }
#endif

    /* convert voltages from fractional measured values to float */
#if SINC_CLOSEDLOOP == ENABLED
    g_sM1Drive.sFocPMSM.fltUDcBus = g_sM1SincFilter.fltUdc;
#else
    g_sM1Drive.sFocPMSM.fltUDcBus = 
        MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
#endif

    /* Disable user application switch */
    g_bM1SwitchAppOnOff = FALSE;

    /* PWM peripheral update */
    MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);

    /* Detects faults */
    M1_FaultDetection();

    /* clear recorded fault state manually from FreeMASTER */
    if(g_sM1Drive.bFaultClearMan)
    {
        /* Clear fault state */
        g_sM1Drive.bFaultClearMan = FALSE;
        g_sM1Drive.sFaultIdCaptured = 0;
    }
}

/*!
 * @brief State initialization routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateInitFast(void)
{
    (void)ui16PWMOutputDebugFlagM1;
    (void)ui16SinSpeedCmdSwitchM1;
    (void)f32AngleM1;
    (void)f32FreqInM1;
    (void)fltSinM1;
    (void)fltSpeedCmdAmplitudeM1;
    (void)fltSpeedCmdTestM1;
    (void)ui16SinCurrentCmdSwitchM1;
    (void)f32AngleCurM1;
    (void)f32FreqInCurM1;
    (void)fltSinCurM1;
    (void)fltCurrentCmdAmplitudeM1;
    (void)fltCurrentCmdTestM1;
    (void)ui16SinPosCmdSwitchM1;
    (void)f32AnglePosM1;
    (void)f32FreqInPosM1;
    (void)f16SinPosM1;
    (void)i32Q16PosCmdAmplitudeM1;
    (void)i32Q16PosCmdTestM1;

	/* Type the code to do when in the INIT state */
    g_sM1Drive.sFocPMSM.sIdPiParams.fltPGain = MID_KP_GAIN;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltIGain = MID_KI_GAIN;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltInErrK_1 = 0.0F;
    g_sM1Drive.sFocPMSM.sIdPiParams.bLimFlag = FALSE;

    g_sM1Drive.sFocPMSM.sIqPiParams.fltPGain = MID_KP_GAIN;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltIGain = MID_KI_GAIN;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltInErrK_1 = 0.0F;
    g_sM1Drive.sFocPMSM.sIqPiParams.bLimFlag = FALSE;

    sIdPiParamsTest.fltPGain = M1_D_KP_GAIN;
    sIdPiParamsTest.fltIGain = M1_D_KI_GAIN;
    sIdPiParamsTest.fltMax = M1_U_MAX;
    sIdPiParamsTest.fltMin = -M1_U_MAX;
  
    sIqPiParamsTest.fltPGain = M1_Q_KP_GAIN;
    sIqPiParamsTest.fltIGain = M1_Q_KI_GAIN;
    sIqPiParamsTest.fltMax = M1_U_MAX;
    sIqPiParamsTest.fltMin = -M1_U_MAX;

    g_sM1Drive.sSpeed.sSpdPiParamsTest.fltPGain = M1_SPEED_PI_PROP_GAIN;
    g_sM1Drive.sSpeed.sSpdPiParamsTest.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    g_sM1Drive.sSpeed.sSpdPiParamsTest.fltMax = M1_SPEED_LOOP_HIGH_LIMIT;
    g_sM1Drive.sSpeed.sSpdPiParamsTest.fltMin = M1_SPEED_LOOP_LOW_LIMIT;

    /* PMSM FOC params */
    g_sM1Drive.sFocPMSM.sIdPiParams.fltPGain = M1_D_KP_GAIN;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltIGain = M1_D_KI_GAIN;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltUpperLim = M1_U_MAX;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltLowerLim = -M1_U_MAX;

    g_sM1Drive.sFocPMSM.sIqPiParams.fltPGain = M1_Q_KP_GAIN;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltIGain = M1_Q_KI_GAIN;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltUpperLim = M1_U_MAX;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltLowerLim = -M1_U_MAX;

    g_sM1Drive.sFocPMSM.ui16SectorSVM = M1_SVM_SECTOR_DEFAULT;
    g_sM1Drive.sFocPMSM.fltDutyCycleLimit = M1_CLOOP_LIMIT;
    
    /* disable dead-time compensation */
    g_sM1Drive.sFocPMSM.bFlagDTComp = FALSE;

    g_sM1Drive.sFocPMSM.fltUDcBus = 0.0F;
    g_sM1Drive.sFocPMSM.fltUDcBusFilt = 0.0F;
    g_sM1Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB0 = M1_UDCB_IIR_B0;
    g_sM1Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltB1 = M1_UDCB_IIR_B1;
    g_sM1Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.fltA1 = M1_UDCB_IIR_A1;
    /* Filter init not to enter to fault */
    g_sM1Drive.sFocPMSM.sUDcBusFilter.fltFltBfrX[0] = 
        (M1_U_DCB_UNDERVOLTAGE / 2.0F) + (M1_U_DCB_OVERVOLTAGE / 2.0F);
    g_sM1Drive.sFocPMSM.sUDcBusFilter.fltFltBfrY[0] =
        (M1_U_DCB_UNDERVOLTAGE / 2.0F) + (M1_U_DCB_OVERVOLTAGE / 2.0F);


    g_sM1Drive.sAlignment.fltUdReq = M1_ALIGN_VOLTAGE;
    g_sM1Drive.sAlignment.ui16Time = M1_ALIGN_DURATION;

    /* Position and speed observer */
    g_sM1Drive.sFocPMSM.sTo.fltPGain = M1_BEMF_DQ_TO_KP_GAIN;
    g_sM1Drive.sFocPMSM.sTo.fltIGain = M1_BEMF_DQ_TO_KI_GAIN;
    g_sM1Drive.sFocPMSM.sTo.fltThGain = M1_BEMF_DQ_TO_THETA_GAIN;

    g_sM1Drive.sFocPMSM.sBemfObsrv.fltIGain = M1_I_SCALE;
    g_sM1Drive.sFocPMSM.sBemfObsrv.fltUGain = M1_U_SCALE;
    g_sM1Drive.sFocPMSM.sBemfObsrv.fltEGain = M1_E_SCALE;
    g_sM1Drive.sFocPMSM.sBemfObsrv.fltWIGain = M1_WI_SCALE;
    g_sM1Drive.sFocPMSM.sBemfObsrv.sCtrl.fltPGain = M1_BEMF_DQ_KP_GAIN;
    g_sM1Drive.sFocPMSM.sBemfObsrv.sCtrl.fltIGain = M1_BEMF_DQ_KI_GAIN;

    g_sM1Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB0 = M1_TO_SPEED_IIR_B0;
    g_sM1Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltB1 = M1_TO_SPEED_IIR_B1;
    g_sM1Drive.sFocPMSM.sSpeedElEstFilt.sFltCoeff.fltA1 = M1_TO_SPEED_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sFocPMSM.sSpeedElEstFilt);
    
    /* Speed params */
    g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = M1_SPEED_PI_PROP_GAIN;
    g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    g_sM1Drive.sSpeed.sSpeedPiParams.fltUpperLim = M1_SPEED_LOOP_HIGH_LIMIT;
    g_sM1Drive.sSpeed.sSpeedPiParams.fltLowerLim = M1_SPEED_LOOP_LOW_LIMIT;

    g_sM1Drive.sSpeed.sSpeedPiParamsDesat.sCoeff.fltPGain = M1_SPEED_PI_PROP_GAIN;
    g_sM1Drive.sSpeed.sSpeedPiParamsDesat.sCoeff.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    g_sM1Drive.sSpeed.sSpeedPiParamsDesat.sCoeff.fltDesatGain = M1_SPEED_PI_DESAT_GAIN;
    g_sM1Drive.sSpeed.sSpeedPiParamsDesat.sCoeff.fltUpperLim = M1_SPEED_LOOP_HIGH_LIMIT;
    g_sM1Drive.sSpeed.sSpeedPiParamsDesat.sCoeff.fltLowerLim = M1_SPEED_LOOP_LOW_LIMIT;

    g_sM1Drive.sSpeed.sSpeedRampParams.fltRampUp = M1_SPEED_RAMP_UP;
    g_sM1Drive.sSpeed.sSpeedRampParams.fltRampDown = M1_SPEED_RAMP_DOWN;

    g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB0 = M1_SPEED_IIR_B0;
    g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltB1 = M1_SPEED_IIR_B1;
    g_sM1Drive.sSpeed.sSpeedFilter.sFltCoeff.fltA1 = M1_SPEED_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedFilter);

    g_sM1Drive.sSpeed.sSpeedDeepFilter.sFltCoeff.fltB0 = M1_SPEED_DEEP_IIR_B0;
    g_sM1Drive.sSpeed.sSpeedDeepFilter.sFltCoeff.fltB1 = M1_SPEED_DEEP_IIR_B1;
    g_sM1Drive.sSpeed.sSpeedDeepFilter.sFltCoeff.fltA1 = M1_SPEED_DEEP_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedDeepFilter);

    g_sM1Drive.sFocPMSM.sCurReqFWFilter.sFltCoeff.fltB0 = M1_CURREQFW_IIR_B0;
    g_sM1Drive.sFocPMSM.sCurReqFWFilter.sFltCoeff.fltB1 = M1_CURREQFW_IIR_B1;
    g_sM1Drive.sFocPMSM.sCurReqFWFilter.sFltCoeff.fltA1 = M1_CURREQFW_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sFocPMSM.sCurReqFWFilter);

    g_sM1Drive.sSpeed.sIqFwdFilter.sFltCoeff.fltB0 = M1_CUR_FWD_IIR_B0;
    g_sM1Drive.sSpeed.sIqFwdFilter.sFltCoeff.fltB1 = M1_CUR_FWD_IIR_B1;
    g_sM1Drive.sSpeed.sIqFwdFilter.sFltCoeff.fltA1 = M1_CUR_FWD_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sIqFwdFilter);

    g_sM1Drive.sPosition.sSpdFwdFilter.sFltCoeff.fltB0 = M1_SPD_FWD_IIR_B0;
    g_sM1Drive.sPosition.sSpdFwdFilter.sFltCoeff.fltB1 = M1_SPD_FWD_IIR_B1;
    g_sM1Drive.sPosition.sSpdFwdFilter.sFltCoeff.fltA1 = M1_SPD_FWD_IIR_A1;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sPosition.sSpdFwdFilter);

    g_sM1Drive.sSpeed.fltSpeedCmd = 0.0F;
    
    g_sM1Drive.sSpeed.fltIqFwdGain = M1_SPEED_LOOP_IQ_FWD_GAIN;

    /* Position params */
    g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32State = 0;
    g_sM1Drive.sPosition.f16SpeedController = 0;
    g_sM1Drive.sPosition.fltSpeedFwd = 0;
    g_sM1Drive.sPosition.fltSpeedFwdNoGain = 0;
    g_sM1Drive.sPosition.fltSpeedRef = 0;
    g_sM1Drive.sPosition.sCurveRef.i32Q16PosCmd = 0;
    g_sM1Drive.sPosition.sCurveRef.i32Q16PosRamp = 0;
    g_sM1Drive.sPosition.sCurveRef.i32Q16PosFilt = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.f32W = M1_QDC_TRAJECTORY_FILTER_FREQ_FRAC;
    g_sM1Drive.sPosition.f16SpeedRefLim = M1_QDC_POSITION_CTRL_LIMIT_FRAC;
    g_sM1Drive.sPosition.fltSpeedRefLim = M1_QDC_POSITION_CTRL_LIMIT;
    g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32RampUp = M1_QDC_POSITION_RAMP_UP_FRAC;
    g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32RampDown = M1_QDC_POSITION_RAMP_DOWN_FRAC;
    g_sM1Drive.sPosition.a32PosGain = M1_QDC_POSITION_CTRL_P_GAIN_FRAC;
    g_sM1Drive.sPosition.i32PosLoopFreq = M1_SLOW_LOOP_FREQ;
    g_sM1Drive.sPosition.fltFreqToAngularSpeedCoeff = (float_t)(2.0*PI*M1_MOTOR_PP);
    g_sM1Drive.sPosition.fltFracToAngularSpeedCoeff = M1_SPEED_FRAC_TO_ANGULAR_COEFF;
    g_sM1Drive.sPosition.fltRpmToAngularSpeedCoeff = M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF;
    g_sM1Drive.sPosition.fltGainSpeedFwd = M1_QDC_POSITION_CTRL_SPEED_FWD_GAIN;
	trajectoryFilterInit(&g_sM1Drive.sPosition.sCurveRef.sTrajFilter);

    /* Scalar control params */
    g_sM1Drive.sScalarCtrl.fltVHzGain = M1_SCALAR_VHZ_FACTOR_GAIN;
    g_sM1Drive.sScalarCtrl.sFreqRampParams.fltRampUp = M1_SCALAR_RAMP_UP;
    g_sM1Drive.sScalarCtrl.sFreqRampParams.fltRampDown = M1_SCALAR_RAMP_DOWN;
    g_sM1Drive.sScalarCtrl.sFreqIntegrator.a32Gain = M1_SCALAR_INTEG_GAIN;
    g_sM1Drive.sScalarCtrl.fltFreqMax = M1_FREQ_MAX;
    
    /* Open loop start up */
    g_sM1Drive.sStartUp.sSpeedIntegrator.a32Gain = M1_SCALAR_INTEG_GAIN;
    g_sM1Drive.sStartUp.f16CoeffMerging = M1_MERG_COEFF;
    g_sM1Drive.sStartUp.fltSpeedCatchUp = M1_MERG_SPEED_TRH;
    g_sM1Drive.sStartUp.fltCurrentStartup = M1_OL_START_I;
    g_sM1Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampUp = M1_OL_START_RAMP_INC;
    g_sM1Drive.sStartUp.sSpeedRampOpenLoopParams.fltRampDown = M1_OL_START_RAMP_INC;
    g_sM1Drive.sStartUp.fltSpeedMax = M1_RAD_MAX;
    g_sM1Drive.sStartUp.bOpenLoop = TRUE;

    /* MCAT cascade control variables */
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.ui16PospeSensor = MCAT_ENC_CTRL;

    /* Timing control and general variables */
    g_sM1Drive.ui16CounterState = 0U;
    g_sM1Drive.ui16TimeFullSpeedFreeWheel = M1_FREEWHEEL_DURATION;
    g_sM1Drive.ui16TimeCalibration = M1_CALIB_DURATION;
    g_sM1Drive.ui16TimeFaultRelease = M1_FAULT_DURATION;
    g_bM1SwitchAppOnOff = FALSE;
    /* Default MCAT control mode after reset */
    g_sM1Drive.eControl = kControlMode_SpeedFOC;

    /* fault set to init states */
    FAULT_CLEAR_ALL(g_sM1Drive.sFaultIdCaptured);
    FAULT_CLEAR_ALL(g_sM1Drive.sFaultIdPending);

    /* fault thresholds */
    g_sM1Drive.sFaultThresholds.fltUDcBusOver = M1_U_DCB_OVERVOLTAGE;
    g_sM1Drive.sFaultThresholds.fltUDcBusUnder = M1_U_DCB_UNDERVOLTAGE;
    g_sM1Drive.sFaultThresholds.fltUDcBusTrip = M1_U_DCB_TRIP;
    g_sM1Drive.sFaultThresholds.fltSpeedOver = M1_N_OVERSPEED_RAD;
    g_sM1Drive.sFaultThresholds.fltSpeedMin = M1_N_MIN_RAD;
    g_sM1Drive.sFaultThresholds.fltSpeedNom = M1_N_NOM_RAD;
    g_sM1Drive.sFaultThresholds.fltUqBemf = M1_E_BLOCK_TRH;
    g_sM1Drive.sFaultThresholds.ui16BlockedPerNum = M1_E_BLOCK_PER;

    /* fault blocked rotor filter */
    g_sM1Drive.msM1BlockedRotorUqFilt.fltLambda = M1_BLOCK_ROT_FAULT_SH;

    /* Defined scaling for FreeMASTER */
    g_fltM1voltageScale = M1_U_MAX;
    g_fltM1currentScale = M1_I_MAX;
    g_fltM1DCBvoltageScale = M1_U_DCB_MAX;
    g_fltM1speedScale = M1_N_MAX;
    g_fltM1speedAngularScale = M1_SPEED_ELEC_ANGULAR_TO_MECH_RPM_COEFF;

    /* Application timing */
    g_sM1Drive.ui16FastCtrlLoopFreq = (M1_PWM_FREQ/M1_FOC_FREQ_VS_PWM_FREQ);
    g_sM1Drive.ui16SlowCtrlLoopFreq = M1_SLOW_LOOP_FREQ;
    
    /* Power Stage characteristic data */
    g_sM1Drive.sFocPMSM.fltPwrStgCharIRange = DTCOMP_I_RANGE;
    g_sM1Drive.sFocPMSM.fltPwrStgCharLinCoeff = DTCOMP_LINCOEFF;

    /* Clear rest of variables  */
    M1_ClearFOCVariables();

    /* Init sensors/actuators pointers */
    /* For PWM driver */
    g_sM1Pwm3ph.psDutyABC         = &(g_sM1Drive.sFocPMSM.sDutyABC);
#if SINC_CLOSEDLOOP == DISABLED
    /* For ADC driver */
    g_sM1AdcSensor.pf16UDcBus     = &(g_sM1Drive.sFocPMSM.f16UDcBus);
    g_sM1AdcSensor.psIABC         = &(g_sM1Drive.sFocPMSM.sIABCFrac);
    g_sM1AdcSensor.pui16SVMSector = &(g_sM1Drive.sFocPMSM.ui16SectorSVM);
    g_sM1AdcSensor.pui16AuxChan   = &(g_sM1Drive.f16AdcAuxSample);
#else
    /* For ADC driver */
    g_sM1AdcSensor.pf16UDcBus     = &(sM1_ADCRslt.f16UDcBus);
    g_sM1AdcSensor.psIABC         = &(sM1_ADCRslt.sIABCFrac);
    g_sM1AdcSensor.pui16SVMSector = &(g_sM1Drive.sFocPMSM.ui16SectorSVM);
    g_sM1AdcSensor.pui16AuxChan   = &(sM1_ADCRslt.f16AdcAuxSample);
#endif

#if POSITION_SENSOR == OPTICAL_ENC  
    /* For QDC driver */
    g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B0 = M1_QDC_SPEED_FILTER_IIR_B0_FRAC;
    g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B1 = M1_QDC_SPEED_FILTER_IIR_B1_FRAC;
    g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32A1 = M1_QDC_SPEED_FILTER_IIR_A1_FRAC;
    MCDRV_QdcSpeedCalInit(&g_sM1QdcSensor);
    MCDRV_QdcToSpeedCalInit(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    MCDRV_TamagawaAbsInit(&g_sM1TamagawaAbs);
    MCDRV_AbsToSpeedCalInit(&g_sM1TamagawaAbs);
#endif
    
    /* INIT_DONE command */
    g_sM1Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}

/*!
 * @brief Stop state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateStopFast(void)
{

    MCDRV_VoltDcBusGet(&g_sM1AdcSensor);
#if POSITION_SENSOR == OPTICAL_ENC  
    /* get rotor position and speed from quadrature encoder sensor */
	if(g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)
	{
		MCDRV_GetRotorCurrentPos(&g_sM1QdcSensor);
		MCDRV_GetRotorCurrentRev(&g_sM1QdcSensor);
		g_sM1Drive.f16PosElEnc = g_sM1QdcSensor.f16PosElec;
	}
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    if(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE)
	{
//		g_sM1PositionCompensate.f16PosElec = g_sM1TamagawaAbs.f16PosElec;         //This part is used for position compensate
//		g_sM1PositionCompensate.fltSpeed = g_sM1Drive.sSpeed.fltSpeedFilt;
//		MCDRV_PositionCompensateCurSampleTrigPosCal(&g_sM1PositionCompensate);
//		if(i16POSTest == 0 )
//		{
	        g_sM1Drive.f16PosElEnc = g_sM1TamagawaAbs.f16PosElec;
//		}
//		else if(i16POSTest ==1)
//		{
//	        g_sM1Drive.f16PosElEnc = g_sM1PositionCompensate.f16CompPosElec;
//		}
    }
#endif


    #if SINC_CLOSEDLOOP == ENABLED
    g_sM1Drive.sFocPMSM.fltUDcBus = g_sM1SincFilter.fltUdc;
    #else
        /* convert voltages from fractional measured values to float */
    g_sM1Drive.sFocPMSM.fltUDcBus = 
        MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
    #endif

    /* Sampled DC-Bus voltage filter */       //because this part have been put in main loop ,so cannot use the IIR filter
//    g_sM1Drive.sFocPMSM.fltUDcBusFilt =
//        GDFLIB_FilterIIR1_FLT(g_sM1Drive.sFocPMSM.fltUDcBus, &g_sM1Drive.sFocPMSM.sUDcBusFilter);
    if(ui16PWMOutputDebugFlagM1 == 1)
    {
        /* PWM peripheral update */
        MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);
        MCDRV_eFlexPwm3PhOutEnable(&g_sM1Pwm3ph);
    }
    else
    {
    	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);
    }
    /* If the user switches on */
    if(g_bM1SwitchAppOnOff != 0)
    {
        /* Set the switch on */
        g_bM1SwitchAppOnOff = TRUE;

        /* Start command */
        g_sM1Ctrl.uiCtrl |= SM_CTRL_START;

    }

    M1_FaultDetection();

    /* If a fault occurred */
    if (g_sM1Drive.sFaultIdPending)
    {
        /* Switches to the FAULT state */
        g_sM1Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

    /* clear recorded fault state manually from FreeMASTER */
    if(g_sM1Drive.bFaultClearMan)
    {
        /* Clear fault state */
        g_sM1Drive.bFaultClearMan = FALSE;
        g_sM1Drive.sFaultIdCaptured = 0;
    }
}

/*!
 * @brief Run state routine called in fast state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunFast(void)
{
    /* get all adc samples - DC-bus voltage, current, bemf and aux sample */
#if POSITION_SENSOR == OPTICAL_ENC
    /* get position and speed from quadrature encoder sensor */
	if(g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)
	{
		MCDRV_GetRotorCurrentPos(&g_sM1QdcSensor);
		MCDRV_GetRotorCurrentRev(&g_sM1QdcSensor);
		g_sM1Drive.f16PosElEnc = g_sM1QdcSensor.f16PosElec;
	}
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    if(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE)
	{
	    g_sM1Drive.f16PosElEnc = g_sM1TamagawaAbs.f16PosElec;
    }
#endif
    /* If the user switches off */
    if (!g_bM1SwitchAppOnOff)
    {
        /* Stop command */
        g_sM1Ctrl.uiCtrl |= SM_CTRL_STOP;
        
        g_sM1Drive.sPosition.sCurveRef.i32Q16PosCmd = 0;
        g_sM1Drive.sPosition.i32Q16PosRef = 0;
        g_sM1Drive.sPosition.i32Q16PosRef_1 = 0;
        g_sM1Drive.sPosition.fltSpeedFwd = 0;
        g_sM1Drive.sPosition.sCurveRef.i32Q16PosFilt = 0;
        g_sM1Drive.sPosition.i32Q16PosFdbk = 0;
            
    }
    /* detect fault */
    M1_FaultDetection();

    /* If a fault occurred */
    if (g_sM1Drive.sFaultIdPending != 0)
    {
        /* Switches to the FAULT state */
        g_sM1Ctrl.uiCtrl |= SM_CTRL_FAULT;
    }

/* !!!  Put this part into Fastloop Critical Code function  !!!
 *
 *
 *
*/

    /* Sampled DC-Bus voltage filter */
//    g_sM1Drive.sFocPMSM.fltUDcBusFilt =
//        GDFLIB_FilterIIR1_FLT(g_sM1Drive.sFocPMSM.fltUDcBus, &g_sM1Drive.sFocPMSM.sUDcBusFilter);

//#if SINC_CLOSEDLOOP == ENABLED
//
//    g_sM1Drive.sFocPMSM.fltUDcBus = g_sM1SincFilter.fltUdc;
//    g_sM1Drive.sFocPMSM.sIABC.fltA = g_sM1SincFilter.fltIa;
//    g_sM1Drive.sFocPMSM.sIABC.fltB = g_sM1SincFilter.fltIb;
//    g_sM1Drive.sFocPMSM.sIABC.fltC = g_sM1SincFilter.fltIc;
//#else
//    /* convert phase currents from fractional measured values to float */
//    g_sM1Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16A, g_fltM1currentScale);
//    g_sM1Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16B, g_fltM1currentScale);
//    g_sM1Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16C, g_fltM1currentScale);
//    /* convert voltages from fractional measured values to float */
//    g_sM1Drive.sFocPMSM.fltUDcBus =
//        MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
//#endif
//
//
//    /* Run sub-state function */
//    s_M1_STATE_RUN_TABLE_FAST[g_eM1StateRun]();
//
//    /* PWM peripheral update */
//    MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);

    /* clear recorded fault state manually from FreeMASTER */
    if(g_sM1Drive.bFaultClearMan)
    {
        /* Clear fault state */
        g_sM1Drive.bFaultClearMan = FALSE;
        g_sM1Drive.sFaultIdCaptured = 0;
    }

}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateFaultSlow(void)
{
#if POSITION_SENSOR == OPTICAL_ENC
	// Get speed from QDC M/T method in slow loop
	MCDRV_QdcSpeedCalUpdate(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
#endif
	/* after fault condition ends wait defined time to clear fault state */
    if (!FAULT_ANY(g_sM1Drive.sFaultIdPending))
    {
        if (--g_sM1Drive.ui16CounterState == 0)
        {
            /* Clear fault state */
            g_sM1Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;
        }
    }
    else
    {
        g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFaultRelease;
    }
}

/*!
 * @brief Fault state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateInitSlow(void)
{
}

/*!
 * @brief Stop state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateStopSlow(void)
{
#if POSITION_SENSOR == OPTICAL_ENC
	// Get speed from QDC M/T method in slow loop
	MCDRV_QdcSpeedCalUpdate(&g_sM1QdcSensor);
	MCDRV_GetRotorDeltaRev(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
//    MCDRV_AbsGetRotorDeltaRev(&g_sM1TamagawaAbs);

#endif

}

/*!
 * @brief Run state routine called in slow state machine
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunSlow(void)
{
#if POSITION_SENSOR == OPTICAL_ENC
	// Get speed from QDC M/T method in slow loop
	MCDRV_QdcSpeedCalUpdate(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
#endif
	/* Run sub-state function */
    s_M1_STATE_RUN_TABLE_SLOW[g_eM1StateRun]();

    g_sM1Drive.eControl_1 = g_sM1Drive.eControl;
}

/*!
 * @brief Transition from Fault to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransFaultStop(void)
{
	/* Type the code to do when going from the FAULT to the INIT state */
//    FAULT_CLEAR_ALL(g_sM1Drive.sFaultIdCaptured);

    /* Clear all FOC variables, init filters, etc. */
    M1_ClearFOCVariables();
}

/*!
 * @brief Transition from Init to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransInitFault(void)
{
    /* type the code to do when going from the INIT to the FAULT state */
    /* disable PWM outputs */
	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFaultRelease;

    g_sM1Drive.sSpeed.fltSpeedCmd = 0.0F;
}

/*!
 * @brief Transition from Init to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransInitStop(void)
{
    /* type the code to do when going from the INIT to the STOP state */
    /* disable PWM outputs */
	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);

    /* Enable Open loop start up */
    g_sM1Drive.sStartUp.bOpenLoop = TRUE;
}

/*!
 * @brief Transition from Stop to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransStopFault(void)
{
    /* type the code to do when going from the STOP to the FAULT state */
    /* load the fault release time to counter */
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFaultRelease;

}

/*!
 * @brief Transition from Stop to Run state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransStopRun(void)
{
    /* type the code to do when going from the STOP to the RUN state */
    /* 50% duty cycle */
    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* PWM duty cycles calculation and update */
    MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);

    /* Clear offset filters */
    MCDRV_Curr3Ph2ShCalibInit(&g_sM1AdcSensor);

    /* Enable PWM output */
//    MCDRV_eFlexPwm3PhOutEnable(&g_sM1Pwm3ph);

    /* pass calibration routine duration to state counter*/
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeCalibration;

    /* Calibration sub-state when transition to RUN */
    g_eM1StateRun = kRunState_Calib;

    /* Acknowledge that the system can proceed into the RUN state */
    g_sM1Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/*!
 * @brief Transition from Run to Fault state
 *
 * @param void  No input parameter
 *
 * @return None
 */

RAM_FUNC_CRITICAL static void M1_TransRunFault(void)
{

    /* type the code to do when going from the RUN to the FAULT state */
    /* disable PWM output */
    MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFaultRelease;

    /* clear over load flag */
    g_sM1Drive.sSpeed.bSpeedPiStopInteg = FALSE;

    g_sM1Drive.sSpeed.fltSpeedCmd = 0.0F;
    g_sM1Drive.sScalarCtrl.fltFreqCmd = 0.0F;
    g_sM1Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    /* Clear actual speed values */
    g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM1Drive.sSpeed.fltSpeed = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedFilt = 0.0F;

    
}

/*!
 * @brief Transition from Run to Stop state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunStop(void)
{
    /* type the code to do when going from the RUN to the STOP state */
    /* disable PWM outputs */
	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);

    g_sM1Drive.sSpeed.fltSpeedCmd = 0.0F;
    g_sM1Drive.sScalarCtrl.fltFreqCmd = 0.0F;
    g_sM1Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD = 0.0F;
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ = 0.0F;
    g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD = 0.0F;

    M1_ClearFOCVariables();

    /* Acknowledge that the system can proceed into the STOP state */
    g_sM1Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}

/*!
 * @brief Calibration process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunCalibFast(void)
{
    /* Type the code to do when in the RUN CALIB sub-state
       performing ADC offset calibration */

    /* call offset measurement */
	MCDRV_Curr3Ph2ShCalib(&g_sM1AdcSensor);

    /* change SVM sector in range <1;6> to measure all AD channel mapping combinations */
    if (++g_sM1Drive.sFocPMSM.ui16SectorSVM > 6)
        g_sM1Drive.sFocPMSM.ui16SectorSVM = 1;
}

/*!
 * @brief Motor identification process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */

RAM_FUNC_CRITICAL static void M1_StateRunMeasureFast(void)
{
    /* Set zero position at all measurements */
    if((g_sMIDCtrl.eState == kMID_Ld) || (g_sMIDCtrl.eState == kMID_Lq) || (g_sMIDCtrl.eState == kMID_Start) || (g_sMIDCtrl.eState == kMID_Rs) || (g_sMIDCtrl.eState == kMID_PwrStgCharact))
    {
        /* Zero position is needed for RL measurement */
        g_sM1Drive.sFocPMSM.f16PosEl = FRAC16(0.0);

        g_sM1Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
        g_sM1Drive.sFocPMSM.sAnglePosEl.fltCos = 1.0F;
    }
    
    /* turn on dead-time compensation in case of Rs measurement */
    g_sM1Drive.sFocPMSM.bFlagDTComp = (g_sMIDCtrl.eState == kMID_Rs);
    
    /* Perform current transformations if voltage control will be done.
     * At other measurements it is done in a current loop calculation */
    if((g_sMIDCtrl.eState == kMID_Ld) || (g_sMIDCtrl.eState == kMID_Lq))
    {
        /* Current transformations */
        GMCLIB_Clark_FLT(&g_sM1Drive.sFocPMSM.sIABC, &g_sM1Drive.sFocPMSM.sIAlBe);
        GMCLIB_Park_FLT(&g_sM1Drive.sFocPMSM.sIAlBe, &g_sM1Drive.sFocPMSM.sAnglePosEl, &g_sM1Drive.sFocPMSM.sIDQ);
    }
    
    /* If electrical parameters are being measured, put external position to FOC */
    if(g_sMIDCtrl.eState == kMID_Mech)
    {
        g_sM1Drive.sFocPMSM.bPosExtOn = (g_sMID.sMIDMech.eState == kMID_MechStartUp);
        g_sM1Drive.sFocPMSM.bOpenLoop = g_sMID.sMIDMech.sStartup.bOpenLoop;
    }
    else
        g_sM1Drive.sFocPMSM.bPosExtOn = TRUE;

    /* Motor parameters measurement state machine */
    MID_SM_StateMachine(&g_sMIDCtrl);

    /* Perform Current control if MID_START or MID_PWR_STG_CHARACT or MID_RS or MID_PP or MID_KE state */
    if((g_sMIDCtrl.eState == kMID_Start) || (g_sMIDCtrl.eState == kMID_PwrStgCharact) || (g_sMIDCtrl.eState == kMID_Rs) || (g_sMIDCtrl.eState == kMID_Pp) || (g_sMIDCtrl.eState == kMID_Ke) || (g_sMIDCtrl.eState == kMID_Mech))
    {    
        /* enable current control loop */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;
    }
    
    /* Perform Voltage control if MID_LD or MID_LQ or START state*/
    else
    {
        /* disable current control loop */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = FALSE;
    }
    
    /* FOC */
    MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);

    /* Force sector to 4 to ensure that currents Ia, Ib will be sensed and Ic calculated */
    g_sM1Drive.sFocPMSM.ui16SectorSVM = 4U;

    /* When Measurement done go to RUN READY sub-state and then to STOP state and reset uw16Enable measurement */
    if(g_sMIDCtrl.uiCtrl & MID_SM_CTRL_STOP_ACK)
    {
        M1_TransRunMeasureReady();
        g_bM1SwitchAppOnOff = FALSE;
        g_sMID.ui16EnableMeasurement = 0U;
    }
}

/*!
 * @brief Ready state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunReadyFast(void)
{
    /* Type the code to do when in the RUN READY sub-state */
    /* Clear actual speed values */
    g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM1Drive.sSpeed.fltSpeed = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM1Drive.sFocPMSM.f16PosElEst = 0;
    g_sM1Drive.sFocPMSM.fltSpeedElEst = 0.0F;

#if FEATURE_MC_INVERTER_OUTPUT_DEBUG_ENABLE
    /**************** Inverter output debug *************/
    if(ui16PWMOutputDebugFlagM1 == 1)
    {
        /* PWM peripheral update */
        MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);
        MCDRV_eFlexPwm3PhOutEnable(&g_sM1Pwm3ph);
    }
    else
    {
    	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);
    }
    /****************************************************/
#endif
    /* MCAT control structure switch */
    switch (g_sM1Drive.eControl)
    {
    case kControlMode_Scalar:
        if (!(g_sM1Drive.sScalarCtrl.fltFreqCmd == 0.0F))
        {
            g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
            g_sM1Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;

            /* Transition to the RUN ALIGN sub-state */
            M1_TransRunReadyAlign();

        }
        break;

    case kControlMode_VoltageFOC:
        if (!(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F ))
        {
            if(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sStartUp.fltSpeedCatchUp * 2.0F;
            else
                g_sM1Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM1Drive.sStartUp.fltSpeedCatchUp * 2.0F);

            /* Transition to the RUN ALIGN sub-state */
            /*Todo g_sM1QdcSensor should include in this file*/
            if(((g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)||(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE))&&(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL))
            {
            	M1_TransRunReadySpin();
            }
            else
            {
            	M1_TransRunReadyAlign();
            }
        }
        break;

    case kControlMode_VoltageOpenloop:
        if ((!(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F ))&&(g_sM1Drive.sSpeed.fltSpeedCmd != 0))
        {
            if(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sStartUp.fltSpeedReq = g_sM1Drive.sSpeed.fltSpeedCmd;
            else
                g_sM1Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM1Drive.sSpeed.fltSpeedCmd);

            M1_TransRunReadyAlign();
        }
        break;

    case kControlMode_CurrentFOC:
        if (!(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F ))
        {
            if(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sStartUp.fltSpeedCatchUp * 2.0F;
            else
                g_sM1Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM1Drive.sStartUp.fltSpeedCatchUp * 2.0F);

            /* Transition to the RUN ALIGN sub-state */
            /*Todo*/
            if(((g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)||(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE))&&(g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL))
            {
            	M1_TransRunReadySpin();
            }
            else
            {
            	M1_TransRunReadyAlign();
            }
        }
        break;

    case kControlMode_CurrentOpenloop:
        if ((!(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F ))&&(g_sM1Drive.sSpeed.fltSpeedCmd != 0))
        {
            if(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sStartUp.fltSpeedReq = g_sM1Drive.sSpeed.fltSpeedCmd;
            else
                g_sM1Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM1Drive.sSpeed.fltSpeedCmd);

            M1_TransRunReadyAlign();
        }
        break;

    case kControlMode_SpeedFOC:
               
    case kControlMode_PositionFOC:
      
    default: 
    	if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
    	{
            /*Todo*/
            if((g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)||(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE) ||(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE))
            {
            	M1_TransRunReadySpin();
            }
            else
            {
            	M1_TransRunReadyAlign();
            }
    	}
    	else if((g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL) &&
                ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) > g_sM1Drive.sFaultThresholds.fltSpeedMin) &&
                (MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) <= g_sM1Drive.sFaultThresholds.fltSpeedNom)))
    	{
    		M1_TransRunReadyAlign();
    	}
        else
        {
            g_sM1Drive.sSpeed.fltSpeedCmd = 0.0F;
        }
    }
}

/*!
 * @brief Alignment process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunAlignFast(void)
{
    /* type the code to do when in the RUN ALIGN sub-state */
    /* When alignment elapsed go to Startup */
    if (--g_sM1Drive.ui16CounterState == 0U)
    {
        if((g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL) && 
           (g_sM1Drive.eControl != kControlMode_Scalar)&&(g_sM1Drive.eControl != kControlMode_VoltageOpenloop)&&(g_sM1Drive.eControl != kControlMode_CurrentOpenloop))
        {
            /* Transition to the RUN kRunState_Spin sub-state */
            M1_TransRunAlignSpin();
        }
        else
        {    
            /* Transition to the RUN kRunState_Startup sub-state */
            M1_TransRunAlignStartup();
        }
    }

    /* If zero speed command go back to Ready */
    if((g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL) && (g_sM1Drive.sSpeed.fltSpeedCmd == 0.0F) && (g_sM1Drive.sScalarCtrl.fltFreqCmd == 0.0F))
        M1_TransRunAlignReady();

    /* clear actual speed values */
    g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM1Drive.sSpeed.fltSpeed = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM1Drive.sFocPMSM.f16PosElEst = 0;
    g_sM1Drive.sFocPMSM.fltSpeedElEst = 0.0F;

    if(g_sM1Drive.ui16CounterState < (M1_ALIGN_DURATION - M1_BOOTSTRAP_CHARGE_DURATION))
    {
		MCS_PMSMAlignment(&g_sM1Drive.sAlignment);
		g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.sAlignment.f16PosAlign;
		MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
    }
}

/*!
 * @brief Start-up process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunStartupFast(void)
{
	frac16_t f16PosDiff = 0; // Position difference between "Merged Position" and "Open-loop Position"
	float_t  fltCosPosDiff;

	/* If f16SpeedCmd = 0, go to Free-wheel state */
    if((g_sM1Drive.sSpeed.fltSpeedCmd==0) && (g_sM1Drive.eControl==kControlMode_SpeedFOC))
        M1_TransRunStartupFreewheel();

    /* Type the code to do when in the RUN STARTUP sub-state */
    /* pass actual estimation position to OL startup structure */
    g_sM1Drive.sStartUp.f16PosEst = g_sM1Drive.sFocPMSM.f16PosElEst;

    /*open loop startup */
    MCS_PMSMOpenLoopStartUp(&g_sM1Drive.sStartUp);

    /* Pass f16SpeedRampOpenloop to f16SpeedRamp*/
    g_sM1Drive.sSpeed.fltSpeedRamp = g_sM1Drive.sStartUp.fltSpeedRampOpenLoop;

    /* Position and speed for FOC */
    g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.sStartUp.f16PosMerged;

    /* MCAT control structure switch */
    switch (g_sM1Drive.eControl)
    {
    case kControlMode_Scalar: 
        /* switch directly to SPIN state */
        M1_TransRunStartupSpin();
        break;

    case kControlMode_VoltageFOC:
        /* pass MCAT required values in run-time */
        g_sM1Drive.sFocPMSM.sUDQReq.fltD = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD;
        g_sM1Drive.sFocPMSM.sUDQReq.fltQ = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ;
        /* FOC */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = FALSE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
        break;  

    case kControlMode_CurrentFOC:
        /* FOC */
        g_sM1Drive.sFocPMSM.sIDQReq.fltD = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD;
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ;
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
        break;

    case kControlMode_VoltageOpenloop:
        /* pass MCAT required values in run-time */
        g_sM1Drive.sFocPMSM.sUDQReq.fltD = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD;
        g_sM1Drive.sFocPMSM.sUDQReq.fltQ = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ;

        if ((!(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ == 0.0F ))&&(g_sM1Drive.sSpeed.fltSpeedCmd != 0))
        {
            if(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sStartUp.fltSpeedReq = g_sM1Drive.sSpeed.fltSpeedCmd;
            else
                g_sM1Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM1Drive.sSpeed.fltSpeedCmd);
        }
        else
        {
        	 M1_TransRunStartupFreewheel();
        }

        /* FOC */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = FALSE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
        break;

    case kControlMode_CurrentOpenloop:

        g_sM1Drive.sFocPMSM.sIDQReq.fltD = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD;
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ;

        if ((!(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ == 0.0F ))&&(g_sM1Drive.sSpeed.fltSpeedCmd != 0))
        {
            if(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ > 0.0F)
                g_sM1Drive.sStartUp.fltSpeedReq = g_sM1Drive.sSpeed.fltSpeedCmd;
            else
                g_sM1Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM1Drive.sSpeed.fltSpeedCmd);
        }
        else
        {
        	 M1_TransRunStartupFreewheel();
        }
        /* FOC */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
        break;


    case kControlMode_SpeedFOC:
      
    case kControlMode_PositionFOC:
        
    default:
        /* Current control loop */
        g_sM1Drive.sFocPMSM.sIDQReq.fltD = 0.0F;

        /* during the open loop start up the values of required Iq current are kept in pre-defined level*/
        if (g_sM1Drive.sSpeed.fltSpeedCmd > 0.0F)
            g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sStartUp.fltCurrentStartup;
        else
            g_sM1Drive.sFocPMSM.sIDQReq.fltQ = MLIB_Neg_FLT(g_sM1Drive.sStartUp.fltCurrentStartup);

        /* Update Iq reference during open-loop and closed-loop merging when sensorless method is used */
        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL)
        {
        	if(g_sM1Drive.sStartUp.f16RatioMerging > 0) // In merging process
        	{
        		fltCosPosDiff = GFLIB_Cos_FLTa((acc32_t)MLIB_Mul_F16(g_sM1Drive.sStartUp.f16RatioMerging, f16PosDiff));
        		g_sM1Drive.sFocPMSM.sIDQReq.fltQ = fltCosPosDiff * g_sM1Drive.sFocPMSM.sIDQReq.fltQ;
        	}
        	else
        	{
        		f16PosDiff = MLIB_Sub_F16(g_sM1Drive.sStartUp.f16PosEst, g_sM1Drive.sStartUp.f16PosGen);
        	}
        }
        

        /* Init Bemf observer if open-loop speed is under SpeedCatchUp/2 */
        if (MLIB_Abs_FLT(g_sM1Drive.sStartUp.fltSpeedRampOpenLoop) <
            (g_sM1Drive.sStartUp.fltSpeedCatchUp / 2.0F))
        {
            AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM1Drive.sFocPMSM.sBemfObsrv);
            AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM1Drive.sFocPMSM.sTo);
        }

        /* FOC */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
               
        /* select source of actual speed value */
        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
            /* pass encoder speed to actual speed value */
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeed = MLIB_ConvSc_FLTsf(g_sM1QdcSensor.sSpeed.f16SpeedFilt, (float_t)(2*FLOAT_PI*M1_FREQ_MAX)); // Get angular electrical speed
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeed.fltSpeed;
#endif    
        else
            /* pass estimated speed to actual speed value */
            g_sM1Drive.sSpeed.fltSpeed = g_sM1Drive.sFocPMSM.fltSpeedElEst;       
    }

    /* switch to close loop  */
    if (!g_sM1Drive.sStartUp.bOpenLoop)
    {
        M1_TransRunStartupSpin();
    }
}

/*!
 * @brief Spin state called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunSpinFast(void)
{
    /* Type the code to do when in the RUN SPIN sub-state */
    /* MCAT control structure switch */
    switch (g_sM1Drive.eControl)
    {
    case kControlMode_Scalar:
        /* scalar control */
        MCS_PMSMScalarCtrl(&g_sM1Drive.sScalarCtrl);

        /* pass required voltages to Bemf Observer to work */
        g_sM1Drive.sFocPMSM.sUDQReq.fltQ = g_sM1Drive.sScalarCtrl.sUDQReq.fltQ;
        g_sM1Drive.sFocPMSM.sUDQReq.fltD = g_sM1Drive.sScalarCtrl.sUDQReq.fltD;
        g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.sScalarCtrl.f16PosElScalar;
        
        /* call voltage FOC to calculate PWM duty cycles */
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);

        /* select source of actual speed value */
//        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1QdcSensor.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeed.fltSpeed;
        
#endif
//        else
        	 /* pass estimated speed to actual speed value */
//        	 g_sM1Drive.sSpeed.fltSpeed = g_sM1Drive.sFocPMSM.fltSpeedElEst;

        /* Sub-state RUN FREEWHEEL */
        if(g_sM1Drive.sScalarCtrl.fltFreqCmd==0.0F)
               M1_TransRunSpinFreewheel();
        break;

    case kControlMode_VoltageFOC:
        /* FOC */
        g_sM1Drive.sFocPMSM.sUDQReq.fltQ = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ;
        g_sM1Drive.sFocPMSM.sUDQReq.fltD = g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltD;
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = FALSE;
        
        /* Pass encoder position to FOC is enabled */
        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
        {
            g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.f16PosElEnc;
            g_sM1Drive.sFocPMSM.bPosExtOn   = TRUE;
        }
        else
        {
            g_sM1Drive.sFocPMSM.bPosExtOn = FALSE;
        }
        
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);

        /* select source of actual speed value */
        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
        {
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1QdcSensor.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeed.fltSpeed;    
#endif
        }
        else
        	 /* pass estimated speed to actual speed value */
        	 g_sM1Drive.sSpeed.fltSpeed = g_sM1Drive.sFocPMSM.fltSpeedElEst;

        /* Sub-state RUN FREEWHEEL */
        if(g_sM1Drive.sMCATctrl.sUDQReqMCAT.fltQ==0.0F)
            M1_TransRunSpinFreewheel();
        break;

    case kControlMode_CurrentFOC: 
        /* current FOC */
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ;
        g_sM1Drive.sFocPMSM.sIDQReq.fltD = g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltD;


        if(ui16SinCurrentCmdSwitchM1 == 1)
        {
        	g_sM1Drive.sFocPMSM.sIDQReq.fltD = fltCurrentCmdTestM1;
            f32AngleCurM1 += MLIB_Mul_F32(FRAC32(2.0*FREQ_CUR_SCALE_M1/M1_FAST_LOOP_FREQ), f32FreqInCurM1);
            fltSinCurM1 = GFLIB_Sin_FLTa((acc32_t)MLIB_Conv_F16l(f32AngleCurM1));
            fltCurrentCmdTestM1 = fltSinCurM1 * fltCurrentCmdAmplitudeM1;
        }

        /* Pass encoder position to FOC is enabled */
//        if(g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)   // To reduce fast loop execution time
//        {
            g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.f16PosElEnc;
            g_sM1Drive.sFocPMSM.bPosExtOn   = TRUE;
//        }
//        else
//        {
//            g_sM1Drive.sFocPMSM.bPosExtOn = FALSE;
//        }
        
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;
        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);

        /* select source of actual speed value */
//        if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
//        {
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1QdcSensor.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeed.fltSpeed;        
#endif
//       }
//        else
			 /* pass estimated speed to actual speed value */
//			 g_sM1Drive.sSpeed.fltSpeed = g_sM1Drive.sFocPMSM.fltSpeedElEst;

        /* Sub-state RUN FREEWHEEL */
        if(g_sM1Drive.sMCATctrl.sIDQReqMCAT.fltQ==0.0F)
            M1_TransRunSpinFreewheel();
        break;

    case kControlMode_SpeedFOC:
    case kControlMode_PositionFOC:
    default: 
        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedRamp) < 
            g_sM1Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
        {
            /* Sub-state RUN FREEWHEEL */
            M1_TransRunSpinFreewheel();
        }
  
        /* Pass encoder position to FOC is enabled */
//        if(g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
//        {
            g_sM1Drive.sFocPMSM.f16PosElExt = g_sM1Drive.f16PosElEnc;       
            g_sM1Drive.sFocPMSM.bPosExtOn   = TRUE;
//        }
//        else
//        {
//            g_sM1Drive.sFocPMSM.bPosExtOn = FALSE;
//        }
        /* FOC */
        g_sM1Drive.sFocPMSM.bCurrentLoopOn = TRUE;

        MCS_PMSMFocCtrl(&g_sM1Drive.sFocPMSM);
        
        /* select source of actual speed value */
//        if(g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_ENC_CTRL)
            /* pass encoder speed to actual speed value */
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1QdcSensor.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeed.fltSpeed;
//            g_sM1Drive.sSpeed.fltSpeed = g_sM1TamagawaAbs.sSpeedEstim.fltSpeedEstim; //Use the speed from TO observer
#endif
//        else
            /* pass estimated speed to actual speed value */
//            g_sM1Drive.sSpeed.fltSpeed = g_sM1Drive.sFocPMSM.fltSpeedElEst;
        
        break;
    }
}

/*!
 * @brief Free-wheel process called in fast state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunFreewheelFast(void)
{
    /* Type the code to do when in the RUN FREEWHEEL sub-state */

    /* clear actual speed values */
    g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM1Drive.sSpeed.fltSpeed = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedRamp = 0.0F;
}

/*!
 * @brief Calibration process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunCalibSlow(void)
{
    if (--g_sM1Drive.ui16CounterState == 0U)
    {
        /* write calibrated offset values */
    	MCDRV_Curr3Ph2ShCalibSet(&g_sM1AdcSensor);

        if(g_sMID.ui16EnableMeasurement != 0U)
            /* To switch to the RUN MEASURE sub-state */
            M1_TransRunCalibMeasure();
        else
            /* To switch to the RUN READY sub-state */
            M1_TransRunCalibReady();
    }
}

/*!
 * @brief Measure state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunMeasureSlow(void)
{
}

/*!
 * @brief Ready state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunReadySlow(void)
{
}

/*!
 * @brief Alignment process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunAlignSlow(void)
{
}

/*!
 * @brief Start-up process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunStartupSlow(void)
{
    if(g_sM1Drive.eControl == kControlMode_SpeedFOC)
    {
        /* actual speed filter */
        g_sM1Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedFilter);

        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) > g_sM1Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM1Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM1Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM1Drive.sFaultThresholds.fltSpeedNom);
        }
    }
}

/*!
 * @brief Spin state called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunSpinSlow(void)
{
    if(g_sM1Drive.eControl == kControlMode_SpeedFOC)
    {
        /* Actual position */
#if POSITION_SENSOR == OPTICAL_ENC
        MCDRV_GetRotorDeltaRev(&g_sM1QdcSensor);
        g_sM1Drive.sPosition.i32Q16PosFdbk = g_sM1QdcSensor.i32Q16DeltaRev;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
        g_sM1Drive.sPosition.i32Q16PosFdbk = g_sM1TamagawaAbs.i32Q16DeltaRev;       
#endif
    	if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
    	{
        /* actual speed filter */
#if POSITION_SENSOR == OPTICAL_ENC
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
#if USEIIRFORBW
    		g_sM1Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedFilter);
//    		g_sM1Drive.sSpeed.fltSpeedDeepFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedDeepFilter);

#else
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#endif
        
#endif
    	}
    	else
    	{
#if USEIIRFORBW
    		g_sM1Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedFilter);
#else
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#endif
    	}

        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) > g_sM1Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM1Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM1Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM1Drive.sFaultThresholds.fltSpeedNom);
        }

        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedRamp) < g_sM1Drive.sFaultThresholds.fltSpeedMin)&&
           (g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
            M1_TransRunSpinFreewheel();

#if FEATURE_MC_LOOP_BANDWIDTH_TEST_ENABLE
        /* Speed loop bandwidth test - for debug only */
        if(ui16SinSpeedCmdSwitchM1 == 1)
        {
        	g_sM1Drive.sSpeed.fltSpeedCmd = fltSpeedCmdTestM1;
        	g_sM1Drive.sSpeed.sSpeedRampParams.fltState = g_sM1Drive.sSpeed.fltSpeedCmd;
        }
#endif

        /* call PMSM speed control */
        g_sM1Drive.sSpeed.bIqPiLimFlag = g_sM1Drive.sFocPMSM.sIqPiParams.bLimFlag;
        MCS_PMSMFocCtrlSpeed(&g_sM1Drive.sSpeed);
#if USEIIRFORBW
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sSpeed.fltIqReq;
#else
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltIqReq, &g_sM1Drive.sFocPMSM.sCurReqFWFilter);
#endif

//        g_sM1Drive.sFocPMSM.fltIqFiltReq = BSF_update(&g_sM1Drive.sPosition.sNotchFilter, g_sM1Drive.sFocPMSM.sIDQReq.fltQ);
    }

    if(g_sM1Drive.eControl == kControlMode_PositionFOC)
    {
    	if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_ENC_CTRL)
    	{
        /* actual speed filter */
#if POSITION_SENSOR == OPTICAL_ENC        
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
#if USEIIRFORBW
    		g_sM1Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedFilter);
//    		g_sM1Drive.sSpeed.fltSpeedDeepFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedDeepFilter);

#else
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#endif
        
#endif
    	}
    	else
    	{
#if USEIIRFORBW
    		g_sM1Drive.sSpeed.fltSpeedFilt = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltSpeed, &g_sM1Drive.sSpeed.sSpeedFilter);
#else
            g_sM1Drive.sSpeed.fltSpeedFilt = g_sM1Drive.sSpeed.fltSpeed;
#endif

    	}
        
        /* pass required speed values lower than nominal speed */
        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) > g_sM1Drive.sFaultThresholds.fltSpeedNom))
        {
            /* set required speed to nominal speed if over speed command > speed nominal */
            if (g_sM1Drive.sSpeed.fltSpeedCmd > 0.0F)
                g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sFaultThresholds.fltSpeedNom;
            else
                g_sM1Drive.sSpeed.fltSpeedCmd = MLIB_Neg_FLT(g_sM1Drive.sFaultThresholds.fltSpeedNom);
        }

        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedRamp) < g_sM1Drive.sFaultThresholds.fltSpeedMin)&&
           (g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
            M1_TransRunSpinFreewheel();
          
        /* Actual position */
#if POSITION_SENSOR == OPTICAL_ENC                 
        MCDRV_GetRotorDeltaRev(&g_sM1QdcSensor);
        g_sM1Drive.sPosition.i32Q16PosFdbk = g_sM1QdcSensor.i32Q16DeltaRev;
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
        g_sM1Drive.sPosition.i32Q16PosFdbk = g_sM1TamagawaAbs.i32Q16DeltaRev;
#endif
#if FEATURE_MC_LOOP_BANDWIDTH_TEST_ENABLE
        if(ui16SinPosCmdSwitchM1 == 1)
        {
        	g_sM1Drive.sPosition.i32Q16PosCmd = i32Q16PosCmdTestM1;
        	g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32State = g_sM1Drive.sPosition.i32Q16PosCmd;
        }
#endif
        /* Set up speed feed-forward environment when switching from speed-control to position-control */
        if(g_sM1Drive.eControl_1 == kControlMode_SpeedFOC)
        {
        	if(g_sM1Drive.sPosition.bIsRandomPosition == TRUE)
        	{
        		g_sM1Drive.sPosition.i32Q16PosRef_1 = g_sM1Drive.sPosition.i32Q16PosCmd;
        	}
        	else
        	{
        		g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32State = g_sM1Drive.sPosition.i32Q16PosCmd;
        		g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M_1 = 0;
        		g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M = 0;
        		g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out = ((int64_t)g_sM1Drive.sPosition.i32Q16PosCmd)<<32;
        		g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out_1 = ((int64_t)g_sM1Drive.sPosition.i32Q16PosCmd)<<32;
        		g_sM1Drive.sPosition.i32Q16PosRef_1 = g_sM1Drive.sPosition.i32Q16PosCmd;
        	}
        }

#if RUN_DEMO_SELECTION
        	if(bPosInitAlginFlag ==0 )
			{
				g_sM1Drive.sPosition.i32Q16PosOffSet = 0;
			}
        	else
        	{
    			g_sM1Drive.sPosition.i32Q16PosOffSet = g_sM1TamagawaAbs.f32PosMechOffsetforAlign>>1;
        	}
#else
			g_sM1Drive.sPosition.i32Q16PosOffSet = g_sM1TamagawaAbs.f32PosMechOffsetforAlign>>1;
#endif

        /* Call PMSM position control */
        MCS_PMSMFocCtrlPosition(&g_sM1Drive.sPosition);
        
        /* Speed command is equal to position controller output */
        g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Drive.sPosition.fltSpeedRef;
        g_sM1Drive.sSpeed.sSpeedRampParams.fltState = g_sM1Drive.sSpeed.fltSpeedCmd; // Bypass speed ramp

        /* Call PMSM speed control */
        g_sM1Drive.sSpeed.bIqPiLimFlag = g_sM1Drive.sFocPMSM.sIqPiParams.bLimFlag;



        MCS_PMSMFocCtrlSpeed(&g_sM1Drive.sSpeed);
#if USEIIRFORBW
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = g_sM1Drive.sSpeed.fltIqReq;
#else
        g_sM1Drive.sFocPMSM.sIDQReq.fltQ = GDFLIB_FilterIIR1_FLT(g_sM1Drive.sSpeed.fltIqReq, &g_sM1Drive.sFocPMSM.sCurReqFWFilter);
#endif


    }
 
}

/*!
 * @brief Free-wheel process called in slow state machine as Run sub state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_StateRunFreewheelSlow(void)
{
    /* wait until free-wheel time passes */
    if (--g_sM1Drive.ui16CounterState == 0)
    {
        /* switch to sub state READY */
        M1_TransRunFreewheelReady();
    }
}

/*!
 * @brief Transition from Calib to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunCalibReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */

    /* set 50% PWM duty cycle */
    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* switch to sub state READY */
    g_eM1StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Calib to Measure state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunCalibMeasure(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN MEASURE sub-state */
    /* Initialise measurement */
  
    /* Set all measurement as inactive */
    g_sMID.sMIDAlignment.ui16Active     = FALSE;
    g_sMID.sMIDPwrStgChar.ui16Active    = FALSE;
    g_sMID.sMIDRs.ui16Active            = FALSE;
    g_sMID.sMIDLs.ui16Active            = FALSE;
    g_sMID.sMIDKe.ui16Active            = FALSE;
    g_sMID.sMIDPp.ui16Active            = FALSE;
    g_sMID.sMIDMech.ui16Active          = FALSE;
    
    /* I/O pointers */
    g_sMID.sIO.pf16PosElExt = &(g_sM1Drive.sFocPMSM.f16PosElExt);
    g_sMID.sIO.pfltId       = &(g_sM1Drive.sFocPMSM.sIDQ.fltD);
    g_sMID.sIO.pfltIq       = &(g_sM1Drive.sFocPMSM.sIDQ.fltQ);
    g_sMID.sIO.pfltIdReq    = &(g_sM1Drive.sFocPMSM.sIDQReq.fltD);
    g_sMID.sIO.pfltIqReq    = &(g_sM1Drive.sFocPMSM.sIDQReq.fltQ);
    g_sMID.sIO.pfltUdReq    = &(g_sM1Drive.sFocPMSM.sUDQReq.fltD);
    g_sMID.sIO.pfltUqReq    = &(g_sM1Drive.sFocPMSM.sUDQReq.fltQ);
    g_sMID.sIO.pfltUDCbus   = &(g_sM1Drive.sFocPMSM.fltUDcBusFilt);
    g_sMID.sIO.pfltEd       = &(g_sM1Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltD);
    g_sMID.sIO.pfltEq       = &(g_sM1Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltQ);
    g_sMID.sIO.pfltSpeedEst = &(g_sM1Drive.sFocPMSM.fltSpeedElEst);
    g_sMID.sIO.pf16PosElEst = &(g_sM1Drive.sFocPMSM.f16PosElEst);
    g_sMID.sIO.pf16PosElExt = &(g_sM1Drive.sFocPMSM.f16PosElExt);

    /* Ls measurement init */
    g_sMID.sMIDLs.fltUdMax   = MLIB_Mul_FLT(MID_K_MODULATION_RATIO, g_sM1Drive.sFocPMSM.fltUDcBusFilt);
    g_sMID.sMIDLs.fltFreqMax = (float_t)g_sM1Drive.ui16FastCtrlLoopFreq / 2U;

    /* Ke measurement init */
    g_sMID.sMIDKe.fltFreqMax = (float_t)g_sM1Drive.ui16FastCtrlLoopFreq / 2U;

    /* Pp measurement init */
    g_sMID.sMIDPp.fltFreqMax = (float_t)g_sM1Drive.ui16FastCtrlLoopFreq / 2U;
    
    /* PwrStg char init */
    g_sMID.sMIDPwrStgChar.ui16NumOfChPnts = MID_CHAR_CURRENT_POINT_NUMBERS;

    /* During the measurement motor is driven open-loop */
    g_sM1Drive.sFocPMSM.bOpenLoop = TRUE; 

    /* Reset DONE & ACK of all MID states */
    g_sMIDCtrl.uiCtrl = 0;

    /* First state in MID state machine will be kMID_Start */
    g_sMIDCtrl.eState = kMID_Start;

    /* Sub-state RUN MEASURE */
    g_eM1StateRun = kRunState_Measure;
}

/*!
 * @brief Transition from Measure to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */

RAM_FUNC_CRITICAL static void M1_TransRunMeasureReady(void)
{
    /* Type the code to do when going from the RUN CALIB to the RUN READY sub-state */
    /* Set off measurement */
    g_sMID.ui16EnableMeasurement = 0;

    /* set 50% PWM duty cycle */
    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    /* disable passing external electrical position to FOC */
    g_sM1Drive.sFocPMSM.bPosExtOn = FALSE;                                   

    /* swith to sub state READY */
    g_eM1StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Ready to Align state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunReadyAlign(void)
{
    /* Type the code to do when going from the RUN kRunState_Ready to the RUN kRunState_Align sub-state */
    /* Alignment duration set-up */
    g_sM1Drive.ui16CounterState = g_sM1Drive.sAlignment.ui16Time;
    /* Counter of half alignment duration */
    g_sM1Drive.sAlignment.ui16TimeHalf = MLIB_ShR_F16(g_sM1Drive.sAlignment.ui16Time, 1);

    /* set required alignment voltage to Ud */
    g_sM1Drive.sFocPMSM.sUDQReq.fltD = g_sM1Drive.sAlignment.fltUdReq;
    g_sM1Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    /* enable passing required position to FOC */
    g_sM1Drive.sFocPMSM.bPosExtOn = TRUE;

    /* disable current FOC */
    g_sM1Drive.sFocPMSM.bCurrentLoopOn = FALSE;
    
    /* enable Open loop mode in main control structure */
    g_sM1Drive.sFocPMSM.bOpenLoop = TRUE;

    if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL)
    {
    	g_sM1Drive.sSpeed.fltIqFwdGain = 0; // Disable Iq feedforward in sensorless control
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = M1_SPEED_PI_PROP_SENSORLESS_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = M1_SPEED_PI_INTEG_SENSORLESS_GAIN;
    }
    else
    {
    	g_sM1Drive.sSpeed.fltIqFwdGain = M1_SPEED_LOOP_IQ_FWD_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = M1_SPEED_PI_PROP_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    }

    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(M1_BOOTSTRAP_DUTY);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(M1_BOOTSTRAP_DUTY);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(M1_BOOTSTRAP_DUTY);

    /* Enable PWM output */
    MCDRV_eFlexPwm3PhOutEnable(&g_sM1Pwm3ph);

    /* Sub-state RUN ALIGN */
    g_eM1StateRun = kRunState_Align;
}

/*!
 * @brief Transition from Align to Startup state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunAlignStartup(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Startup sub-state */
    /* initialize encoder driver */
#if POSITION_SENSOR == OPTICAL_ENC  
	MCDRV_GetRotorInitPos(&g_sM1QdcSensor, MLIB_Conv_F32s(g_sM1Drive.sAlignment.f16PosAlign));
	MCDRV_GetRotorInitRev(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
	MCDRV_AbsGetRotorInitPos(&g_sM1TamagawaAbs, MLIB_Conv_F32s(g_sM1Drive.sAlignment.f16PosAlign));
	MCDRV_AbsGetRotorInitRev(&g_sM1TamagawaAbs);
#endif 
    /* clear application parameters */
    M1_ClearFOCVariables();

    /* pass required speed to open loop start-up structure */
    if (g_sM1Drive.sSpeed.fltSpeedCmd > 0.0F)
        g_sM1Drive.sStartUp.fltSpeedReq = g_sM1Drive.sStartUp.fltSpeedCatchUp;
    else
        g_sM1Drive.sStartUp.fltSpeedReq = MLIB_Neg_FLT(g_sM1Drive.sStartUp.fltSpeedCatchUp);

    /* enable position merge in openloop startup  */
    if((g_sM1Drive.eControl!= kControlMode_VoltageOpenloop)&&(g_sM1Drive.eControl!= kControlMode_CurrentOpenloop))
    {
    	g_sM1Drive.sStartUp.bMergeFlag = TRUE;
    }
    else
    {
    	g_sM1Drive.sStartUp.bMergeFlag = FALSE;
    }

    /* enable Open loop mode in main control structure */
    g_sM1Drive.sStartUp.bOpenLoop = TRUE;
    g_sM1Drive.sFocPMSM.bOpenLoop = TRUE;

    /* enable Open loop mode in FOC module */
    g_sM1Drive.sFocPMSM.bPosExtOn = TRUE;

    g_sM1Drive.sFocPMSM.ui16SectorSVM = M1_SVM_SECTOR_DEFAULT;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedFilter);
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedDeepFilter);
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sFocPMSM.sCurReqFWFilter);

    /* Go to sub-state RUN STARTUP */
    g_eM1StateRun = kRunState_Startup;
}

/*!
 * @brief Transition from Align to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunAlignSpin(void)
{
    /* Type the code to do when going from the RUN STARTUP to the RUN SPIN sub-state */
 #if POSITION_SENSOR == OPTICAL_ENC
    /* initialize encoder driver */
	MCDRV_GetRotorInitPos(&g_sM1QdcSensor, MLIB_Conv_F32s(g_sM1Drive.sAlignment.f16PosAlign));

    /* Ensure the feedback position is reset to zero whenever state machine goes to spin */
    MCDRV_GetRotorInitRev(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
	MCDRV_AbsGetRotorInitPos(&g_sM1TamagawaAbs, MLIB_Conv_F32s(g_sM1Drive.sAlignment.f16PosAlign));
	MCDRV_AbsGetRotorInitRev(&g_sM1TamagawaAbs);
#endif
    /* Ensure position command starts from zero */
    g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32State = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M_1 = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out_1 = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out = 0;
  
    g_sM1Drive.sFocPMSM.bPosExtOn = TRUE;                                        /* enable passing external electrical position from encoder to FOC */
    g_sM1Drive.sFocPMSM.bOpenLoop = FALSE;                                       /* disable parallel running openloop and estimator */
  
    g_sM1Drive.sFocPMSM.ui16SectorSVM    = M1_SVM_SECTOR_DEFAULT;
    g_sM1Drive.sFocPMSM.sIDQReq.fltD     = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQReq.fltQ     = 0.0F;
    
    g_sM1Drive.sSpeed.fltSpeedRamp = 0;
    g_sM1Drive.sSpeed.fltSpeedRamp_1 = 0;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrX[0] = 0;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrY[0] = 0;

    M1_ClearFOCVariables();

    /* To switch to the RUN SPIN sub-state */
    g_eM1StateRun = kRunState_Spin;
}

RAM_FUNC_CRITICAL static void M1_TransRunReadySpin(void)
{
    g_sM1Drive.sFocPMSM.bPosExtOn = TRUE;                                        /* enable passing external electrical position from encoder to FOC */
    g_sM1Drive.sFocPMSM.bOpenLoop = FALSE;                                       /* disable parallel running openloop and estimator */

    g_sM1Drive.sFocPMSM.ui16SectorSVM    = M1_SVM_SECTOR_DEFAULT;
    g_sM1Drive.sFocPMSM.sIDQReq.fltD     = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQReq.fltQ     = 0.0F;

    M1_ClearFOCVariables();

    /* Ensure the feedback position is reset whenever state machine goes to spin */
#if POSITION_SENSOR == OPTICAL_ENC
    MCDRV_GetRotorInitRev(&g_sM1QdcSensor);
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    MCDRV_AbsGetRotorInitRev(&g_sM1TamagawaAbs);
//    MCDRV_AbsGetRotorInitPos(&g_sM1TamagawaAbs, MLIB_Conv_F32s(g_sM1Drive.sAlignment.f16PosAlign));
	g_sM1Drive.sPosition.i32Q16PosOffSet = 0;
#endif

    /* Ensure position command starts from zero */
    g_sM1Drive.sPosition.sCurveRef.sPosRamp.f32State = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64M_1 = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out_1 = 0;
    g_sM1Drive.sPosition.sCurveRef.sTrajFilter.i64Out = 0;

    g_sM1Drive.sSpeed.fltSpeedRamp = 0;
    g_sM1Drive.sSpeed.fltSpeedRamp_1 = 0;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrX[0] = 0;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrY[0] = 0;

    /* Enable PWM output */
    MCDRV_eFlexPwm3PhOutEnable(&g_sM1Pwm3ph);

    if(g_sM1Drive.sMCATctrl.ui16PospeSensor == MCAT_SENSORLESS_CTRL)
    {
    	g_sM1Drive.sSpeed.fltIqFwdGain = 0; // Disable Iq feedforward in sensorless control
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = M1_SPEED_PI_PROP_SENSORLESS_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = M1_SPEED_PI_INTEG_SENSORLESS_GAIN;
    }
    else
    {
    	g_sM1Drive.sSpeed.fltIqFwdGain = M1_SPEED_LOOP_IQ_FWD_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltPGain = M1_SPEED_PI_PROP_GAIN;
    	g_sM1Drive.sSpeed.sSpeedPiParams.fltIGain = M1_SPEED_PI_INTEG_GAIN;
    }

    /* To switch to the RUN SPIN sub-state */
    g_eM1StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Align to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunAlignReady(void)
{
    /* Type the code to do when going from the RUN kRunState_Align to the RUN kRunState_Ready sub-state */
    
    /* Clear FOC accumulators */
    M1_ClearFOCVariables();

    /* Go to sub-state RUN READY */
    g_eM1StateRun = kRunState_Ready;
}

/*!
 * @brief Transition from Startup to Spin state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunStartupSpin(void)
{
    /* Type the code to do when going from the RUN kRunState_Startup to the RUN kRunState_Spin sub-state */
    /* for FOC control switch open loop off in DQ observer */
    if(g_sM1Drive.eControl!=kControlMode_Scalar)
    {    
        g_sM1Drive.sFocPMSM.bPosExtOn = FALSE; /* disable passing external electrical position to FOC */
        g_sM1Drive.sFocPMSM.bOpenLoop = FALSE; /* disable parallel running open-loop and estimator */
    }

    g_sM1Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = g_sM1Drive.sFocPMSM.sIDQReq.fltQ;
    g_sM1Drive.sSpeed.sSpeedRampParams.fltState = g_sM1Drive.sSpeed.fltSpeedFilt;
    g_sM1Drive.sSpeed.fltSpeedRamp = g_sM1Drive.sSpeed.fltSpeedFilt;
    g_sM1Drive.sSpeed.fltSpeedRamp_1 = g_sM1Drive.sSpeed.fltSpeedFilt;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrX[0] = 0;
    g_sM1Drive.sSpeed.sIqFwdFilter.fltFltBfrY[0] = 0;

    /* To switch to the RUN kRunState_Spin sub-state */
    g_eM1StateRun = kRunState_Spin;
}

/*!
 * @brief Transition from Startup to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunStartupFreewheel(void)
{
	MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);

    /* Free-wheel duration set-up */
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFullSpeedFreeWheel;

    /* enter FREEWHEEL sub-state */
    g_eM1StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Spin to Free-wheel state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunSpinFreewheel(void)
{
    /* Type the code to do when going from the RUN SPIN to the RUN FREEWHEEL sub-state */
    /* set 50% PWM duty cycle */
    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

    g_sM1Drive.sFocPMSM.ui16SectorSVM = M1_SVM_SECTOR_DEFAULT;

    MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);

    /* Generates a time gap before the alignment to assure the rotor is not rotating */
    g_sM1Drive.ui16CounterState = g_sM1Drive.ui16TimeFullSpeedFreeWheel;

    g_sM1Drive.sFocPMSM.sIDQReq.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQReq.fltQ = 0.0F;

    g_sM1Drive.sFocPMSM.sUDQReq.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;

    g_sM1Drive.sFocPMSM.sIAlBe.fltAlpha = 0.0F;
    g_sM1Drive.sFocPMSM.sIAlBe.fltBeta = 0.0F;
    g_sM1Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0.0F;
    g_sM1Drive.sFocPMSM.sUAlBeReq.fltBeta = 0.0F;

    /* enter FREEWHEEL sub-state */
    g_eM1StateRun = kRunState_Freewheel;
}

/*!
 * @brief Transition from Free-wheel to Ready state
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_TransRunFreewheelReady(void)
{
    /* Type the code to do when going from the RUN kRunState_FreeWheel to the RUN kRunState_Ready sub-state */
    /* clear application parameters */
    M1_ClearFOCVariables();

    MCDRV_eFlexPwm3PhOutDisable(&g_sM1Pwm3ph);

    /* Sub-state RUN READY */
    g_eM1StateRun = kRunState_Ready;
}

/*!
 * @brief Clear FOc variables in global variable
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_ClearFOCVariables(void)
{
    g_sM1Drive.sAlignment.ui16TimeHalf = 0U;

    /* Clear FOC variables */
    g_sM1Drive.sFocPMSM.sIABC.fltA = 0.0F;
    g_sM1Drive.sFocPMSM.sIABC.fltB = 0.0F;
    g_sM1Drive.sFocPMSM.sIABC.fltC = 0.0F;
    g_sM1Drive.sFocPMSM.sIAlBe.fltAlpha = 0.0F;
    g_sM1Drive.sFocPMSM.sIAlBe.fltBeta = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQ.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQ.fltQ = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQReq.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQReq.fltQ = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQError.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sIDQError.fltQ = 0.0F;
    g_sM1Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);
    g_sM1Drive.sFocPMSM.sUAlBeReq.fltAlpha = 0.0F;
    g_sM1Drive.sFocPMSM.sUAlBeReq.fltBeta = 0.0F;
    g_sM1Drive.sFocPMSM.sUDQReq.fltD = 0.0F;
    g_sM1Drive.sFocPMSM.sUDQReq.fltQ = 0.0F;
    g_sM1Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
    g_sM1Drive.sFocPMSM.sAnglePosEl.fltCos = 0.0F;
    g_sM1Drive.sFocPMSM.sAnglePosEl.fltSin = 0.0F;
    g_sM1Drive.sFocPMSM.sAnglePosEl.fltCos = 0.0F;
    g_sM1Drive.sFocPMSM.sIdPiParams.bLimFlag = FALSE;
    g_sM1Drive.sFocPMSM.sIqPiParams.bLimFlag = FALSE;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM1Drive.sFocPMSM.sIdPiParams.fltIAccK_1 = 0.0F;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    g_sM1Drive.sFocPMSM.sIqPiParams.fltIAccK_1 = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sFocPMSM.sSpeedElEstFilt);
    g_sM1Drive.sFocPMSM.bIdPiStopInteg = FALSE;
    g_sM1Drive.sFocPMSM.bIqPiStopInteg = FALSE;

    /* Clear Speed control state variables */
    g_sM1Drive.sSpeed.sSpeedRampParams.fltState = 0.0F;
    g_sM1Drive.sSpeed.fltSpeed = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedFilt = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedError = 0.0F;
    g_sM1Drive.sSpeed.fltSpeedRamp = 0.0F;
    g_sM1Drive.sSpeed.sSpeedPiParams.fltIAccK_1 = 0.0F;
    g_sM1Drive.sSpeed.sSpeedPiParams.bLimFlag = FALSE;
    g_sM1Drive.sSpeed.sSpeedFilter.fltFltBfrX[0] = 0.0F;
    g_sM1Drive.sSpeed.sSpeedFilter.fltFltBfrY[0] = 0.0F;
    g_sM1Drive.sSpeed.bSpeedPiStopInteg = FALSE;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedFilter);

    g_sM1Drive.sSpeed.sSpeedDeepFilter.fltFltBfrX[0] = 0.0F;
    g_sM1Drive.sSpeed.sSpeedDeepFilter.fltFltBfrY[0] = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sSpeed.sSpeedDeepFilter);

    g_sM1Drive.sPosition.sSpdFwdFilter.fltFltBfrX[0] = 0.0F;
    g_sM1Drive.sPosition.sSpdFwdFilter.fltFltBfrY[0] = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sPosition.sSpdFwdFilter);
    g_sM1Drive.sFocPMSM.sCurReqFWFilter.fltFltBfrX[0] = 0.0F;
    g_sM1Drive.sFocPMSM.sCurReqFWFilter.fltFltBfrY[0] = 0.0F;
    GDFLIB_FilterIIR1Init_FLT(&g_sM1Drive.sFocPMSM.sCurReqFWFilter);
    PIControllerDesatInit(&g_sM1Drive.sSpeed.sSpeedPiParamsDesat);
    PIController_Init(&g_sM1Drive.sSpeed.sSpdPiParamsTest);
    PIController_Init(&sIdPiParamsTest);
    PIController_Init(&sIqPiParamsTest);

    /* Init Blocked rotor filter */
    GDFLIB_FilterMAInit_FLT(0.0F, &g_sM1Drive.msM1BlockedRotorUqFilt);

    /* Clear Scalar control variables */
    g_sM1Drive.sScalarCtrl.fltFreqRamp = 0.0F;
    g_sM1Drive.sScalarCtrl.f16PosElScalar = 0.0F;
    g_sM1Drive.sScalarCtrl.sUDQReq.fltD = 0.0F;
    g_sM1Drive.sScalarCtrl.sUDQReq.fltQ = 0.0F;
    g_sM1Drive.sScalarCtrl.sFreqIntegrator.f32IAccK_1 = 0;
    g_sM1Drive.sScalarCtrl.sFreqIntegrator.f16InValK_1 = 0;
    g_sM1Drive.sScalarCtrl.sFreqRampParams.fltState = 0.0F;

    /* Clear Startup variables */
    g_sM1Drive.sStartUp.f16PosMerged = 0;
    g_sM1Drive.sStartUp.f16PosEst = 0;
    g_sM1Drive.sStartUp.f16PosGen = 0;
    g_sM1Drive.sStartUp.f16RatioMerging = 0;
    g_sM1Drive.sStartUp.fltSpeedRampOpenLoop = 0.0F;
    g_sM1Drive.sStartUp.fltSpeedReq = 0.0F;
    g_sM1Drive.sStartUp.sSpeedIntegrator.f32IAccK_1 = 0;
    g_sM1Drive.sStartUp.sSpeedIntegrator.f16InValK_1 = 0;
    g_sM1Drive.sStartUp.sSpeedRampOpenLoopParams.fltState = 0.0F;

    BSF_init(&g_sM1Drive.sPosition.sNotchFilter);

    /* Clear BEMF and Tracking observers state variables */
    AMCLIB_PMSMBemfObsrvDQInit_A32fff(&g_sM1Drive.sFocPMSM.sBemfObsrv);
    AMCLIB_TrackObsrvInit_A32af(ACC32(0.0), &g_sM1Drive.sFocPMSM.sTo);
}

/*!
 * @brief Fault detention routine - check various faults
 *
 * @param void  No input parameter
 *
 * @return None
 */
RAM_FUNC_CRITICAL static void M1_FaultDetection(void)
{
    /* Clearing actual faults before detecting them again  */
    /* Clear all faults */
    FAULT_CLEAR_ALL(g_sM1Drive.sFaultIdPending);

#if POSITION_SENSOR == TAMAGAWA_ABS_ENC
    /* Tamagawa abs error */
    if(g_sM1TamagawaAbs.ui16Status)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_TAMAGAWA_ABS);
    }
#endif
    /* GD3000:   reset */
    if (g_sM1GD3000.ui8ResetRequest)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_IN_RESET);
    }

    /* GD3000:   DC-bus over-current */
    if (g_sM1GD3000.sStatus.uStatus0.B.overCurrent)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_OC);
    }

    /* GD3000:   VLS under voltage */
    if (g_sM1GD3000.sStatus.uStatus0.B.lowVls)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_VLS_UV);
        g_sM1GD3000.ui8ResetRequest = 1;
    }

    /* GD3000:   over-temperature */
    if (g_sM1GD3000.sStatus.uStatus0.B.overTemp)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_OT);
    }

    /* GD3000:   phase error */
    if (g_sM1GD3000.sStatus.uStatus0.B.phaseErr)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_PHASE_ERR);
    }

    /* GD3000:   Desaturation detected */
    if (g_sM1GD3000.sStatus.uStatus0.B.desaturation)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_GD3000_DESAT);
    }

    /* Fault:   DC-bus over-current */
    if (MCDRV_eFlexPwm3PhFaultGet(&g_sM1Pwm3ph))
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_I_DCBUS_OVER);
    }

    /* Fault:   DC-bus over-voltage */
    if (g_sM1Drive.sFocPMSM.fltUDcBusFilt > g_sM1Drive.sFaultThresholds.fltUDcBusOver)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_U_DCBUS_OVER);
    }

    /* Fault:   DC-bus under-voltage */
    if (g_sM1Drive.sFocPMSM.fltUDcBusFilt < g_sM1Drive.sFaultThresholds.fltUDcBusUnder)
    {
        FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_U_DCBUS_UNDER);
        g_sM1GD3000.ui8ResetRequest = 1;
    }

    /* Check only in SPEED_FOC control, RUN state, kRunState_Spin and kRunState_FreeWheel sub-states */
    if((g_sM1Drive.eControl==kControlMode_SpeedFOC) && 
       (g_sM1Ctrl.eState==kSM_AppRun) && 
       (g_eM1StateRun==kRunState_Spin || g_eM1StateRun==kRunState_Freewheel) &&
       (g_sM1Drive.sMCATctrl.ui16PospeSensor==MCAT_SENSORLESS_CTRL))
    {
        /* Fault: Overload  */
        float_t fltSpeedFiltAbs = MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedFilt);
        float_t fltSpeedRampAbs = MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedRamp);

        if ((fltSpeedFiltAbs < g_sM1Drive.sFaultThresholds.fltSpeedMin) &&
            (fltSpeedRampAbs > g_sM1Drive.sFaultThresholds.fltSpeedMin) &&
            (g_sM1Drive.sSpeed.bSpeedPiStopInteg == TRUE))
        {
            FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_LOAD_OVER);
        }

        /* Fault: Over-speed  */
        if ((MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedFilt) > g_sM1Drive.sFaultThresholds.fltSpeedOver) &&
            (MLIB_Abs_FLT(g_sM1Drive.sSpeed.fltSpeedCmd) > g_sM1Drive.sFaultThresholds.fltSpeedMin))
        {
            FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_SPEED_OVER);
        }

        /* Fault: Blocked rotor detection */
        /* filter of bemf Uq voltage */
        g_sM1Drive. fltBemfUqAvg = GDFLIB_FilterMA_FLT(g_sM1Drive.sFocPMSM.sBemfObsrv.sEObsrv.fltQ,
                                                      &g_sM1Drive.msM1BlockedRotorUqFilt);
        /* check the bemf Uq voltage threshold only in kRunState_Spin - RUN state */
        if ((MLIB_Abs_FLT(g_sM1Drive.fltBemfUqAvg) < g_sM1Drive.sFaultThresholds.fltUqBemf) &&
            (g_eM1StateRun == kRunState_Spin))
            g_sM1Drive.ui16BlockRotorCnt++;
        else
            g_sM1Drive.ui16BlockRotorCnt = 0U;
        /* for bemf voltage detected above limit longer than defined period number set blocked rotor fault*/
        if (g_sM1Drive.ui16BlockRotorCnt > g_sM1Drive.sFaultThresholds.ui16BlockedPerNum)
        {
            FAULT_SET(g_sM1Drive.sFaultIdPending, FAULT_ROTOR_BLOCKED);
            g_sM1Drive.ui16BlockRotorCnt = 0U;
        }
    }
    /* pass fault to Fault ID Captured */
    g_sM1Drive.sFaultIdCaptured |= g_sM1Drive.sFaultIdPending;
}

RAM_FUNC_CRITICAL void M1_FastLoopCriticalCode(void)
{
#if POSITION_SENSOR == OPTICAL_ENC
	if(g_sM1QdcSensor.bPosAbsoluteFlag == TRUE)
	{
		MCDRV_GetRotorCurrentPos(&g_sM1QdcSensor);
		MCDRV_GetRotorCurrentRev(&g_sM1QdcSensor);
		g_sM1Drive.f16PosElEnc = g_sM1QdcSensor.f16PosElec;
	}
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    if(g_sM1TamagawaAbs.bPosAbsoluteFlag == TRUE)
	{
	    g_sM1Drive.f16PosElEnc = g_sM1TamagawaAbs.f16PosElec;
    }
#endif

#if FEATURE_MC_LOOP_BANDWIDTH_TEST_ENABLE
    /* Speed loop bandwidth test - for debug only */
    if(ui16SinSpeedCmdSwitchM1 == 1)
    {
    	f32AngleM1 += MLIB_Mul_F32(FRAC32(2.0*FREQ_SCALE_M1/M1_FAST_LOOP_FREQ), f32FreqInM1);
    	fltSinM1 = GFLIB_Sin_FLTa((acc32_t)MLIB_Conv_F16l(f32AngleM1));
    	fltSpeedCmdTestM1 = fltSinM1 * fltSpeedCmdAmplitudeM1 + fltSpeedoffset;
    }


    /* Position loop bandwidth test - For debug only */
    if(ui16SinPosCmdSwitchM1 == 1)
    {
      	f32AnglePosM1 += MLIB_Mul_F32(FRAC32(2.0*FREQ_POS_SCALE_M1/M1_FAST_LOOP_FREQ), f32FreqInPosM1);
       	f16SinPosM1 = GFLIB_Sin_F16(MLIB_Conv_F16l(f32AnglePosM1));
       	i32Q16PosCmdTestM1 = ((int64_t)f16SinPosM1 * i32Q16PosCmdAmplitudeM1)>>15; // Q1.15 * Q16.16 = Q17.31 => Q16.16
    }
#endif

#if SINC_CLOSEDLOOP == ENABLED
//    SYSTICK_START_COUNT();
    g_sM1Drive.sFocPMSM.fltUDcBus = g_sM1SincFilter.fltUdc;
    g_sM1Drive.sFocPMSM.sIABC.fltA = g_sM1SincFilter.fltIa;
    g_sM1Drive.sFocPMSM.sIABC.fltB = g_sM1SincFilter.fltIb;
    g_sM1Drive.sFocPMSM.sIABC.fltC = g_sM1SincFilter.fltIc;
#else
    /* convert phase currents from fractional measured values to float */
    g_sM1Drive.sFocPMSM.sIABC.fltA = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16A, g_fltM1currentScale);
    g_sM1Drive.sFocPMSM.sIABC.fltB = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16B, g_fltM1currentScale);
    g_sM1Drive.sFocPMSM.sIABC.fltC = MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.sIABCFrac.f16C, g_fltM1currentScale);
    /* convert voltages from fractional measured values to float */
    g_sM1Drive.sFocPMSM.fltUDcBus =
        MLIB_ConvSc_FLTsf(g_sM1Drive.sFocPMSM.f16UDcBus, g_fltM1DCBvoltageScale);
#endif
//    SYSTICK_STOP_COUNT(ui32ExeTime_fastloopM1);
    /* Sampled DC-Bus voltage filter */
    g_sM1Drive.sFocPMSM.fltUDcBusFilt =
        GDFLIB_FilterIIR1_FLT(g_sM1Drive.sFocPMSM.fltUDcBus, &g_sM1Drive.sFocPMSM.sUDcBusFilter);

    if(g_sM1Ctrl.eState == kSM_AppRun)
    {
        /* Run sub-state function */
        s_M1_STATE_RUN_TABLE_FAST[g_eM1StateRun]();



        /* PWM peripheral update */
        MCDRV_eFlexPwm3PhDutyUpdate(&g_sM1Pwm3ph);
    }

}


