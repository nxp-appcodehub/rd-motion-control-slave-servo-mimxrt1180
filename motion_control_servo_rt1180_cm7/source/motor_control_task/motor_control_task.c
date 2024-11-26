/*
    * Copyright 2022 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/

#include "mc_periph_init.h"
#include "motor_control_task.h"
#include "m1_sm_ref_sol.h"
#include "m2_sm_ref_sol.h"
#include "fsl_common.h"
#include "freemaster.h"
#include "mlib.h"
#include "gdflib.h"
#include "fsl_rgpio.h"
#include "pin_mux.h"
#include "mc_common.h"
#include "mc_periph_init.h"
#include "SPC010728Y00.h"
#include "api_motorcontrol.h"
#define DEBUGMOTOR 1       //This macro are used to debug motors individually
/*****************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile static int8_t i8M1_PWMPeriodCnt, i8M2_PWMPeriodCnt;
volatile static uint16_t ui16ExeTime_fastloopM1, ui16ExeTime_fastloopM2, ui16ExeTime_fastloopM3, ui16ExeTime_fastloopM4;
uint16_t ui16CntSlowloopM1, ui16CntSlowloopM2, ui16CntSlowloopM3, ui16CntSlowloopM4;
volatile uint16_t ui16CntFastloopM1, ui16CntFastloopM2, ui16CntFastloopM3, ui16CntFastloopM4;
static mc_motor_status_t sMotor1Status,sMotor2Status;
volatile int32_t i32M1DeltaCmd,i32M1PosCmd,i32M1PosCmd_1 ,i32M2DeltaCmd,i32M2PosCmd,i32M2PosCmd_1;
volatile uint32_t ui32ExeTime_fastloopM1,ui32ForceM1Cnt,ui32ForceM2Cnt;
uint16_t u16M1AvgCnt,u16M2AvgCnt,u16SwitchMotorSlowLoop,u16AverageCNT;
bool_t bOpen_M1_DMA,bOpen_M1_Abs,bOpen_M1_Uart,bOpen_M1and2_FastLoop,bOpen_M1and2_Slowloop,bOpen_M2_DMA,bOpen_M2_Abs,bOpen_M2_Uart,bPWMUpdateSelFlag;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Qtimer interrupt. Get M1 and M2 commands from master.
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void  CMD_Receive_handler(void)
{
#if DEBUGMOTOR
#else

    // Clear flag
    TMR1->CHANNEL[0].SCTRL &= ~TMR_SCTRL_TCF_MASK;
    u16AverageCNT = (float)g_sM1Cmd.u32SyncLoopCycle/62.50;
    if(u16AverageCNT <= 1)
    	u16AverageCNT = 1;
    // Get M1 Cmd
	switch(g_sM1Cmd.eControlMethodSel)
	{
	case kMC_ScalarControl:
		g_sM1Drive.eControl = kControlMode_Scalar;
		g_sM1Drive.sScalarCtrl.fltVHzGain = g_sM1Cmd.uSpeed_pos.sScalarParam.fltScalarControlVHzGain;
		g_sM1Drive.sScalarCtrl.fltFreqCmd = g_sM1Cmd.uSpeed_pos.sScalarParam.fltScalarControlFrequency;
		break;
	case kMC_FOC_SpeedControl:
		g_sM1Drive.eControl = kControlMode_SpeedFOC;
		g_sM1Drive.sSpeed.fltSpeedCmd = g_sM1Cmd.uSpeed_pos.fltSpeed * M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF;
		break;
	case kMC_FOC_PositionControl:
	default:
		g_sM1Drive.eControl = kControlMode_PositionFOC;
// Don't use the random position mode; All of the commands need to be possessed by filter.
//		if(g_sM1Cmd.uSpeed_pos.sPosParam.bIsRandomPosition == true)
//		{
//			g_sM1Drive.sPosition.bIsRandomPosition = TRUE; // Don't generate trajectory in position loop
//
//		}
//		else
//		{
//			g_sM1Drive.sPosition.bIsRandomPosition = FALSE;
//		}

//To guarantee the smoothness of the command, need equalize the command; freq_sync:1k, freq_slowloop:8k;
		i32M1PosCmd_1 = i32M1PosCmd;
		i32M1PosCmd = g_sM1Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw;
		i32M1DeltaCmd = (i32M1PosCmd -i32M1PosCmd_1)/u16AverageCNT; // delta position/8
//		g_sM1Drive.sPosition.i32Q16PosCmd = g_sM1Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw;
		break;
	}
	g_sM1Drive.sMCATctrl.ui16PospeSensor = g_sM1Cmd.ui16MUPospeSensor; //Kevin add
	// Turn on/off motor
	if(g_sM1Cmd.eAppSwitch == kMC_App_On)
	{
		g_bM1SwitchAppOnOff = TRUE;
	}
	else
	{
		g_bM1SwitchAppOnOff = FALSE;
	}

    // Get M2 Cmd
	switch(g_sM2Cmd.eControlMethodSel)
	{

	case kMC_ScalarControl:
		g_sM2Drive.eControl = kControlMode_Scalar;
		g_sM2Drive.sScalarCtrl.fltVHzGain = g_sM2Cmd.uSpeed_pos.sScalarParam.fltScalarControlVHzGain;
		g_sM2Drive.sScalarCtrl.fltFreqCmd = g_sM2Cmd.uSpeed_pos.sScalarParam.fltScalarControlFrequency;
		break;
	case kMC_FOC_SpeedControl:
		g_sM2Drive.eControl = kControlMode_SpeedFOC;
		g_sM2Drive.sSpeed.fltSpeedCmd = g_sM2Cmd.uSpeed_pos.fltSpeed * M1_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF;
		break;
	case kMC_FOC_PositionControl:
	default:
		g_sM2Drive.eControl = kControlMode_PositionFOC;
//		if(g_sM2Cmd.uSpeed_pos.sPosParam.bIsRandomPosition == true)
//		{
//			g_sM2Drive.sPosition.bIsRandomPosition = TRUE; // Don't generate trajectory in position loop
//
//		}
//		else
//		{
//			g_sM2Drive.sPosition.bIsRandomPosition = FALSE;
//		}
		i32M2PosCmd_1 = i32M2PosCmd;
		i32M2PosCmd = g_sM2Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw;
		i32M2DeltaCmd = (i32M2PosCmd -i32M2PosCmd_1)/u16AverageCNT;
//		g_sM2Drive.sPosition.i32Q16PosCmd = g_sM2Cmd.uSpeed_pos.sPosParam.uPosition.i32Raw;
		break;
	}
	g_sM2Drive.sMCATctrl.ui16PospeSensor = g_sM2Cmd.ui16MUPospeSensor;
	// Turn on/off motor
	if(g_sM2Cmd.eAppSwitch == kMC_App_On)
	{
		g_bM2SwitchAppOnOff = TRUE;
	}
	else
	{
		g_bM2SwitchAppOnOff = FALSE;
	}

    sMotor1Status.g_eM1StateRun = g_eM1StateRun;
    sMotor1Status.eControlMethodSel = g_sM1Drive.eControl;

    sMotor1Status.sSlow.eAppSwitch = g_bM1SwitchAppOnOff;
    sMotor1Status.sSlow.fltSpeed = g_sM1Drive.sSpeed.fltSpeedFilt*M1_SPEED_ELEC_ANGULAR_TO_MECH_RPM_COEFF;
    sMotor1Status.sSlow.uPosition.i32Raw = g_sM1Drive.sPosition.i32Q16PosFdbk;
    sMotor1Status.sFast.eFaultStatus = g_sM1Drive.sFaultIdPending;
    sMotor1Status.sFast.eMotorState = g_sM1Ctrl.eState;
    sMotor1Status.sFast.fltIa = g_sM1Drive.sFocPMSM.sIABC.fltA;
    sMotor1Status.sFast.fltIb = g_sM1Drive.sFocPMSM.sIABC.fltB;
    sMotor1Status.sFast.fltIc = g_sM1Drive.sFocPMSM.sIABC.fltC;
    sMotor1Status.sFast.fltVDcBus = g_sM1Drive.sFocPMSM.fltUDcBusFilt;
    sMotor1Status.sFast.fltValpha = g_sM1Drive.sFocPMSM.sUAlBeReq.fltAlpha;
    sMotor1Status.sFast.fltVbeta = g_sM1Drive.sFocPMSM.sUAlBeReq.fltBeta;


    sMotor2Status.g_eM1StateRun = g_eM2StateRun;
    sMotor2Status.eControlMethodSel = g_sM2Drive.eControl;

    sMotor2Status.sSlow.eAppSwitch = g_bM2SwitchAppOnOff;
    sMotor2Status.sSlow.fltSpeed = g_sM2Drive.sSpeed.fltSpeedFilt*M1_SPEED_ELEC_ANGULAR_TO_MECH_RPM_COEFF;
    sMotor2Status.sSlow.uPosition.i32Raw = g_sM2Drive.sPosition.i32Q16PosFdbk;
    sMotor2Status.sFast.eFaultStatus = g_sM2Drive.sFaultIdPending;
    sMotor2Status.sFast.eMotorState = g_sM2Ctrl.eState;
    sMotor2Status.sFast.fltIa = g_sM2Drive.sFocPMSM.sIABC.fltA;
    sMotor2Status.sFast.fltIb = g_sM2Drive.sFocPMSM.sIABC.fltB;
    sMotor2Status.sFast.fltIc = g_sM2Drive.sFocPMSM.sIABC.fltC;
    sMotor2Status.sFast.fltVDcBus = g_sM2Drive.sFocPMSM.fltUDcBusFilt;
    sMotor2Status.sFast.fltValpha = g_sM2Drive.sFocPMSM.sUAlBeReq.fltAlpha;
    sMotor2Status.sFast.fltVbeta = g_sM2Drive.sFocPMSM.sUAlBeReq.fltBeta;

    MC_SetMotorStatusFromISR(&sMotor1Status, &sMotor2Status);

	u16M1AvgCnt = 0;
	u16M2AvgCnt = 0;

	if(bPWMUpdateSelFlag ==0 )
	{
		bPWMUpdateSelFlag = 1;
		if(PWM4->SM[0].CNT < 4125 )
		{
			g_sM1Pwm3ph.ui16PWMUpdateSel = 1;
			g_sM2Pwm3ph.ui16PWMUpdateSel = 0;
		}
		else
		{
			g_sM1Pwm3ph.ui16PWMUpdateSel = 0;
			g_sM2Pwm3ph.ui16PWMUpdateSel = 1;
		}
	}

#endif

    __DSB();
}


/*!
 * @brief Motor1 and 2 slow loop ISR where speed and position loops are executed.
 *
 * @param None
 *
 * @return None
 */

RAM_FUNC_CRITICAL void M1_M2_slowloop_handler_PWM(void)
{
	// Clear flag
    PWM4->SM[3].STS = PWM_STS_RF_MASK | PWM_STS_CMPF_MASK;

/* When an PWM CMP interrupt is entered, enable the GPIO to observe the timing distribution of the interrupt.*/
	if(bOpen_M1and2_Slowloop==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}

//	MCDRV_AbsToSpeedCalUpdate(&g_sM1TamagawaAbs);    //when use the speed from TO observer ,open this part
//	MCDRV_AbsToSpeedCalUpdate(&g_sM2TamagawaAbs);
//     Every interrupt only run M1 or M2 slow loop, which determined by u16SwitchMotorSlowLoop;
    if( u16SwitchMotorSlowLoop == 0)
    {
#if DEBUGMOTOR
#else
    	/*
    	 * Motor commands are received every 1ms, while slowloop's are calculated every 125us.
    	 * The received commands are divided into equal parts.
    	 */
    	g_sM1Drive.sPosition.i32Q16PosCmd = u16M1AvgCnt *i32M1DeltaCmd + i32M1PosCmd_1;
    	u16M1AvgCnt ++;
//    	g_sM1Drive.sPosition.i32Q16PosCmd = i32M1PosCmd;
#endif

		MCDRV_AbsGetRotorDeltaRev(&g_sM1TamagawaAbs);

		/* M1 Slow StateMachine call */
		SM_StateMachineSlow(&g_sM1Ctrl);
		u16SwitchMotorSlowLoop++;
    }
    else if (u16SwitchMotorSlowLoop == 1)
    {
#if DEBUGMOTOR
#else
    	g_sM2Drive.sPosition.i32Q16PosCmd = u16M2AvgCnt *i32M2DeltaCmd + i32M2PosCmd_1;
    	u16M2AvgCnt ++;
//    	g_sM2Drive.sPosition.i32Q16PosCmd = i32M2PosCmd;
#endif

        MCDRV_AbsGetRotorDeltaRev(&g_sM2TamagawaAbs);
        /* M2 Slow StateMachine call */
        SM_StateMachineSlow(&g_sM2Ctrl);
        u16SwitchMotorSlowLoop = 0;
    }
	if(bOpen_M1and2_Slowloop==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}

    __DSB();
}

/*!
 * @brief Motor1&2 fast loop ISR where current loop is executed.This is use the SD-ADC for current sample;
 *
 * @param None
 *
 * @return None
 */

RAM_FUNC_CRITICAL void M1_M2_fastloop_handler_PWM(void)
{

    SYSTICK_START_COUNT();
	// Clear flag
    PWM2->SM[3].STS = PWM_STS_RF_MASK | PWM_STS_CMPF_MASK;
	if(bOpen_M1and2_FastLoop==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}



	if((SINC1->CHANNEL[0].CSR && SINC_CSR_RDRS_MASK)>>12 ==0)  //if the sinc data is ready  , run M1&M2 ACR
	{

	    MCDRV_Get3PhCurrents_without_sw_decimation(&g_sM1SincFilter);
	    MCDRV_GetDcBusVoltage_without_sw_decimation(&g_sM1SincFilter);

	    MCDRV_Get3PhCurrents_without_sw_decimation(&g_sM2SincFilter);

	}
//    SYSTICK_START_COUNT();
    M1_FastLoopCriticalCode();
//    SYSTICK_STOP_COUNT(ui32ForceM1Cnt);
    M2_FastLoopCriticalCode();
    SYSTICK_STOP_COUNT(ui32ExeTime_fastloopM1);
	if(bOpen_M1and2_FastLoop==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}

    if (++ui16CntFastloopM1 >= (uint16_t)(M1_FAST_LOOP_FREQ))
    {
        ui16CntFastloopM1 = 0;
	    RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED1_GPIO, BOARD_INITPINS_USER_LED1_GPIO_PIN_MASK);
    }


    /* Recorder of FreeMASTER*/
    FMSTR_Recorder(0);
    __DSB();
}



# if 0
/*!
 * @brief Motor1 fast loop ISR where current loop is executed. This is use the SAR-ADC to sample current/udc;
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M1_ADC_fastloop_handler(void)
{
    bool_t bADCStatus;
    
    RESET_TIMER1();

    // Clear flag
    ADC1->STAT = ADC_STAT_TCOMP_INT_MASK;
    bADCStatus = MCDRV_AdcRawDataGet(&g_sM1AdcSensor); // Read all the results from the FIFOs

#if SINC_CLOSEDLOOP == DISABLED
    
    MCDRV_GetRotorCurrentRev(&g_sM1QdcSensor);
    MCDRV_GetRotorDeltaRev(&g_sM1QdcSensor);

    /*========================= State machine begin========================================================*/
    /* M1 State machine */
    SM_StateMachineFast(&g_sM1Ctrl);
    /*========================= State machine end =========================================================*/

    /* Recorder of FreeMASTER*/
    FMSTR_Recorder(0);

    if (++ui16CntFastloopM1 >= (uint16_t)(M1_FAST_LOOP_FREQ))
    {
        ui16CntFastloopM1 = 0;
        RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED2_GPIO, BOARD_INITPINS_USER_LED2_GPIO_PIN_MASK);
    }
    ui16ExeTime_fastloopM1 = READ_TIMER1();
#endif

    __DSB();
}


/*!
 * @brief Motor2 fast loop ISR where current loop is executed. This is use the SAR-ADC;
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M2_ADC_fastloop_handler(void)
{
    bool_t bADCStatus;

    RESET_TIMER1();
    // Clear flag
    ADC2->STAT = ADC_STAT_TCOMP_INT_MASK;
    bADCStatus = MCDRV_AdcRawDataGet(&g_sM2AdcSensor); // Read all the results from the FIFOs

#if SINC_CLOSEDLOOP == DISABLED
    
    MCDRV_GetRotorCurrentRev(&g_sM2QdcSensor);
    MCDRV_GetRotorDeltaRev(&g_sM2QdcSensor);

    /*========================= State machine begin========================================================*/
    /* M1 State machine */
        /* 
          PWM update needs writing register 7 times, reading register 2 times
          QDC: reading register 7 times
        */
    SM_StateMachineFast(&g_sM2Ctrl); 
    /*========================= State machine end =========================================================*/
    
    
      
    if(++ui16CntFastloopM2 >= (uint16_t)(M2_FAST_LOOP_FREQ))
    {
        ui16CntFastloopM2 = 0;
        RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED3_GPIO, BOARD_INITPINS_USER_LED3_GPIO_PIN_MASK);
    }
    ui16ExeTime_fastloopM2 = READ_TIMER1();
    
#endif
    // RGPIO_ClearPinsOutput(BOARD_INITPINS_TP151_GPIO, BOARD_INITPINS_TP151_GPIO_PIN_MASK);
    __DSB();
}

#endif

/*!
 * @brief Motor1 PWM VAL5 compare ISR. This is to make sure SINC can be triggered after 4 PWM period time has been passed
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M1_PWM_CMP_handler(void)
{
    // Clear flag
    PWM4->SM[0].STS = PWM_STS_RF_MASK | PWM_STS_CMPF_MASK;

	if(bOpen_M1_Abs==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    if(i8M1_PWMPeriodCnt < 4)
    {
        i8M1_PWMPeriodCnt++;
    }
    else if(i8M1_PWMPeriodCnt == 4)
    {
        SINC1->MCR |= SINC_MCR_MEN_MASK; // master enable
        i8M1_PWMPeriodCnt = 5;
    }
    else
    {
//    	if(g_sM1TamagawaAbs.pGetPWMForceFlag() == true)
//    	{
//    		if(++ui32ForceM1Cnt == 8000)
//    		{
//    			ui32ForceM1Cnt = 0;
//    		}
//    	}
#if POSITION_SENSOR == TAMAGAWA_TAMA_ENC
        g_sM1TamagawaAbs.sRequestFrame.ui8ControlField = READ_DATA_ALL;
        MCDRV_AbsRequestData(&g_sM1TamagawaAbs);
#endif
    }
	if(bOpen_M1_Abs==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    __DSB();
}

/*!
 * @brief Motor2 PWM VAL5 compare ISR. This is to make sure SINC can be triggered after 4 PWM period time has been passed
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M2_PWM_CMP_handler(void)
{
    // Clear flag
    PWM2->SM[0].STS = PWM_STS_RF_MASK | PWM_STS_CMPF_MASK;
	if(bOpen_M2_Abs==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    if(i8M2_PWMPeriodCnt < 4)
    {
        i8M2_PWMPeriodCnt++;
    }
    else if(i8M2_PWMPeriodCnt == 4)
    {
        SINC2->MCR |= SINC_MCR_MEN_MASK; // master enable
        i8M2_PWMPeriodCnt = 5;
    }
    else
    {
//    	if(g_sM2TamagawaAbs.pGetPWMForceFlag() == true)
//    	{
//    		if(++ui32ForceM2Cnt == 8000)
//    		{
//    			ui32ForceM2Cnt = 0;
//    		}
//    	}
#if POSITION_SENSOR == TAMAGAWA_TAMA_ENC
        g_sM2TamagawaAbs.sRequestFrame.ui8ControlField = READ_DATA_ALL;
        MCDRV_AbsRequestData(&g_sM2TamagawaAbs);
#endif
    }
	if(bOpen_M2_Abs==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    __DSB();
}

RAM_FUNC_CRITICAL void M1_TAMA_ABS_LPUART_TX_handler(void)
{
	if(bOpen_M1_Uart==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    g_sM1TamagawaAbs.LPUART->CTRL &= ~LPUART_CTRL_TCIE_MASK; // disable interrupt
	if(g_sM1TamagawaAbs.ui8RequestAckFlag == SEND_REQEUST)
	{
		g_sM1TamagawaAbs.pReceiveEnable();
		g_sM1TamagawaAbs.LPUART->CTRL |= LPUART_CTRL_RE_MASK; // Enable UART receive
	}
	if(bOpen_M1_Uart==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    SDK_ISR_EXIT_BARRIER;
}

RAM_FUNC_CRITICAL void M2_TAMA_ABS_LPUART_TX_handler(void)
{

	if(bOpen_M2_Uart==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    g_sM2TamagawaAbs.LPUART->CTRL &= ~LPUART_CTRL_TCIE_MASK; // disable interrupt

	if(g_sM2TamagawaAbs.ui8RequestAckFlag == SEND_REQEUST)
	{
        g_sM2TamagawaAbs.pReceiveEnable();
        g_sM2TamagawaAbs.LPUART->CTRL |= LPUART_CTRL_RE_MASK; // Enable UART receive
	}
	if(bOpen_M2_Uart==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    SDK_ISR_EXIT_BARRIER;
}

/*!
 * @brief Motor1 DMA ISR. Get the ABS encoder position information from DMA
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M1_TAMA_ABS_DMA_RX_handler(void)
{
	if(bOpen_M1_DMA==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    SYSTICK_START_COUNT();
    DMA4->TCD[g_sM1TamagawaAbs.ui8DMAChannel].CH_INT |= DMA_CH_INT_INT_MASK; // Clear major loop interrupt flag
   
    g_sM1TamagawaAbs.LPUART->CTRL &= ~LPUART_CTRL_RE_MASK; // Disable UART receive

    MCDRV_AbsGetDataResponse(&g_sM1TamagawaAbs);
	MCDRV_AbsGetRotorCurrentPos(&g_sM1TamagawaAbs);
	SYSTICK_STOP_COUNT(ui32ForceM1Cnt);
	MCDRV_AbsGetRotorCurrentRev(&g_sM1TamagawaAbs);
	MCDRV_AbsSpeedCalculation(&g_sM1TamagawaAbs);

	if(bOpen_M1_DMA==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}

    SDK_ISR_EXIT_BARRIER;
}
/*!
 * @brief Motor2 DMA ISR. Get the ABS encoder position information from DMA
 *
 * @param None
 *
 * @return None
 */
RAM_FUNC_CRITICAL void M2_TAMA_ABS_DMA_RX_handler(void)
{
	if(bOpen_M2_DMA==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    DMA4->TCD[g_sM2TamagawaAbs.ui8DMAChannel].CH_INT |= DMA_CH_INT_INT_MASK; // Clear major loop interrupt flag
   
    g_sM2TamagawaAbs.LPUART->CTRL &= ~LPUART_CTRL_RE_MASK; // Disable UART receive

    MCDRV_AbsGetDataResponse(&g_sM2TamagawaAbs);
	MCDRV_AbsGetRotorCurrentPos(&g_sM2TamagawaAbs);
	MCDRV_AbsGetRotorCurrentRev(&g_sM2TamagawaAbs);
    MCDRV_AbsSpeedCalculation(&g_sM2TamagawaAbs);
	if(bOpen_M2_DMA==1)
	{
		RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED0_GPIO, BOARD_INITPINS_USER_LED0_GPIO_PIN_MASK);
	}
    SDK_ISR_EXIT_BARRIER;
}

