/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mc_periph_init.h"
#include "fsl_lpadc.h"
#include "fsl_common.h"
#include "fsl_xbar.h"
#include "fsl_lpspi.h"
#include "fsl_lpuart.h"
#include "fsl_rgpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "SPC010728Y00.h"
#include "fsl_memory.h"
#include "fsl_edma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Structure for current and voltage measurement*/
mcdrv_adc_t g_sM1AdcSensor, g_sM2AdcSensor;

/* Structure for 3-phase PWM MC driver */
mcdrv_pwm3ph_pwma_t g_sM1Pwm3ph, g_sM2Pwm3ph;

/* Structure for Encoder driver */
mcdrv_qdc_block_t g_sM1QdcSensor, g_sM2QdcSensor;

/* Structure for GD3000 driver */
mcdrv_GD3000_t g_sM1GD3000, g_sM2GD3000;

/* Structure to observe ADC sampled currents & voltages when FOC uses currents & voltages from SINC filter */
adc_results_t sM1_ADCRslt, sM2_ADCRslt;

/* Structure for Tamagawa Absolute encoder driver */
mcdrv_tamagawa_abs_t g_sM1TamagawaAbs, g_sM2TamagawaAbs;

/* Structure for Sinc filter driver */
mcdrv_sinc_t g_sM1SincFilter,g_sM2SincFilter;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief   void M1_M2_InitADC(void)
 *           - Initialization of the ADC peripheral
 *
 * @param   void
 *
 * @return  none
 */
static void M1_M2_InitADC(void)
{
    bool_t bInitRslt;

    lpadc_config_t mLpadcConfigStruct;

    LPADC_GetDefaultConfig(&mLpadcConfigStruct);

    mLpadcConfigStruct.enableAnalogPreliminary = true;
    mLpadcConfigStruct.referenceVoltageSource = kLPADC_ReferenceVoltageAlt2;
    mLpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage1024;
    mLpadcConfigStruct.FIFO1Watermark = 1U;

    LPADC_Init(ADC1, &mLpadcConfigStruct);
    LPADC_Init(ADC2, &mLpadcConfigStruct);

    LPADC_SetOffsetCalibrationMode(ADC1, kLPADC_OffsetCalibration16bitMode);
    LPADC_DoOffsetCalibration(ADC1);
    LPADC_SetOffsetCalibrationMode(ADC1, kLPADC_OffsetCalibration12bitMode);
    LPADC_DoOffsetCalibration(ADC1);
    LPADC_DoAutoCalibration(ADC1);

    LPADC_SetOffsetCalibrationMode(ADC2, kLPADC_OffsetCalibration16bitMode);
    LPADC_DoOffsetCalibration(ADC2);
    LPADC_SetOffsetCalibrationMode(ADC2, kLPADC_OffsetCalibration12bitMode);
    LPADC_DoOffsetCalibration(ADC2);
    LPADC_DoAutoCalibration(ADC2);

    /* Offset filter window */
    g_sM1AdcSensor.ui16OffsetFiltWindow = M1_ADC_OFFSET_WINDOW;
    g_sM1AdcSensor.sDual_adc_sequence.ui16Average = M1_ADC_AVG_NUM;
    g_sM1AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum = TRIGGER_0;
    g_sM1AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum = CMD_1;
    g_sM1AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum = CMD_2;
    g_sM1AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum = TRIGGER_0;
    g_sM1AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum = CMD_5;
    g_sM1AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum = CMD_8;

    g_sM1AdcSensor.sDual_adc_sequence.ui16IaChannelInfo = M1_IA_INFO;
    g_sM1AdcSensor.sDual_adc_sequence.ui16IbChannelInfo = M1_IB_INFO;
    g_sM1AdcSensor.sDual_adc_sequence.ui16IcChannelInfo = M1_IC_INFO;
    g_sM1AdcSensor.sDual_adc_sequence.ui16UdcChannelInfo = M1_UDC_INFO;

    MCDRV_ADCDriverInit(&g_sM1AdcSensor);
    bInitRslt = MCDRV_Curr3PhDcBusVoltChanAssignInit(&g_sM1AdcSensor);
    while (bInitRslt == FALSE); // Debug only

    /* Offset filter window */
    g_sM2AdcSensor.ui16OffsetFiltWindow = M2_ADC_OFFSET_WINDOW;
    g_sM2AdcSensor.sDual_adc_sequence.ui16Average = M2_ADC_AVG_NUM;
    g_sM2AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum = TRIGGER_1;
    g_sM2AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum = CMD_3;
    g_sM2AdcSensor.sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum = CMD_4;
    g_sM2AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum = TRIGGER_1;
    g_sM2AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum = CMD_6;
    g_sM2AdcSensor.sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum = CMD_9;

    g_sM2AdcSensor.sDual_adc_sequence.ui16IaChannelInfo = M2_IA_INFO;
    g_sM2AdcSensor.sDual_adc_sequence.ui16IbChannelInfo = M2_IB_INFO;
    g_sM2AdcSensor.sDual_adc_sequence.ui16IcChannelInfo = M2_IC_INFO;
    g_sM2AdcSensor.sDual_adc_sequence.ui16UdcChannelInfo = M2_UDC_INFO;

    MCDRV_ADCDriverInit(&g_sM2AdcSensor);
    bInitRslt = MCDRV_Curr3PhDcBusVoltChanAssignInit(&g_sM2AdcSensor);
    while (bInitRslt == FALSE); // Debug only

    /* Enable the trigger completion interrupt. */
    LPADC_EnableInterrupts(ADC1, kLPADC_Trigger0CompletionInterruptEnable);
//    EnableIRQ(ADC1_IRQn);
#if SINC_CLOSEDLOOP == ENABLED
//    NVIC_SetPriority(ADC1_IRQn, 4U);       //Use the SAR-ADC interrupt for debug
#else
    NVIC_SetPriority(ADC1_IRQn, 1U);
#endif

//    LPADC_EnableInterrupts(ADC2, kLPADC_Trigger1CompletionInterruptEnable);
//    EnableIRQ(ADC2_IRQn);
#if SINC_CLOSEDLOOP == ENABLED
//    NVIC_SetPriority(ADC2_IRQn, 4U);
#else
    NVIC_SetPriority(ADC2_IRQn, 1U);
#endif
}

/*!
 * @brief   void M1_InitQtimer1_0(void)
 *           - Initialization of the TMR1 peripheral
 *           - When receive the ECAT sync signal,will delay pre-set time trigger the Qtimer CMP interrupt;
 *           - M1_DELAY_AFTER_ETHERCAT_SYNC is the macro of  delay time
 *           - Use the interrupt to update the motor1 and motor2 commands from master
 *
 * @param   void
 *
 * @return  none
 */
static void M1_InitQtimer1_0(void) // For motor1 slow loop
{
    CLOCK_EnableClock(kCLOCK_Qtimer1);

    TMR1->CHANNEL[0].CTRL = TMR_CTRL_PCS(0x8)|TMR_CTRL_SCS(0x0) | TMR_CTRL_CM(6) | TMR_CTRL_LENGTH_MASK; // IP bus clock

    TMR1->CHANNEL[0].COMP1 = (BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT/1000000) * M1_DELAY_AFTER_ETHERCAT_SYNC;

    TMR1->CHANNEL[0].SCTRL |= TMR_SCTRL_TCFIE_MASK;

    NVIC_SetPriority(TMR1_IRQn, 4);
    NVIC_EnableIRQ(TMR1_IRQn);
}

/*!
 * @brief   void M1_InitQtimer2_0(void)
 *           - Initialization of the TMR1 peripheral
 *           - Not used; Stand by
 * @param   void
 *
 * @return  none
 */
static void M2_InitQtimer2_0(void)
{
    CLOCK_EnableClock(kCLOCK_Qtimer2);

    TMR2->CHANNEL[0].CTRL = TMR_CTRL_PCS(0xB) | TMR_CTRL_CM(0) | TMR_CTRL_LENGTH_MASK; // IP bus clock/8

    TMR2->CHANNEL[0].COMP1 = BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / (8.0 * M1_SLOW_LOOP_FREQ);

    TMR2->CHANNEL[0].SCTRL |= TMR_SCTRL_TCFIE_MASK;

}

/*!
 * @brief   uint8_t LPSPI3_sendData(uint8_t ui8Data)
 *           - SPI send data function;
 * @param
 *
 * @return
 */
static uint8_t LPSPI3_sendData(uint8_t ui8Data)
{
    uint32_t ui32Status;

    LPSPI_WriteData(LPSPI3, ui8Data);

    do
    {
        ui32Status = LPSPI_GetStatusFlags(LPSPI3);
    } while ((ui32Status & kLPSPI_TransferCompleteFlag) == 0);

    LPSPI_ClearStatusFlags(LPSPI3, kLPSPI_TransferCompleteFlag);

    do
    {
        ui32Status = LPSPI_GetStatusFlags(LPSPI3);
    } while ((ui32Status & kLPSPI_RxDataReadyFlag) == 0);

    return LPSPI_ReadData(LPSPI3);
}

static void M1_GD3000_reset_setVal(uint8_t ui8Val)
{
    RGPIO_PinWrite(BOARD_INITLPSPI3_M1_GD3000_RESET_GPIO, BOARD_INITLPSPI3_M1_GD3000_RESET_GPIO_PIN, ui8Val);
}

static void M1_GD3000_enable_setVal(uint8_t ui8Val)
{
    RGPIO_PinWrite(BOARD_INITLPSPI3_M1_GD3000_EN_GPIO, BOARD_INITLPSPI3_M1_GD3000_EN_GPIO_PIN, ui8Val);
}
/*!
 * @brief   void M1_InitLPSPI(void)
 *           - Initialization of the SPI3 peripheral for initial M1_GD3000;
 * @param
 *
 * @return
 */
static void M1_InitLPSPI(void)
{

    lpspi_master_config_t sSPI3Config;

    LPSPI_MasterGetDefaultConfig(&sSPI3Config);
    sSPI3Config.baudRate = 4000000;
    sSPI3Config.bitsPerFrame = 8;
    sSPI3Config.cpha = kLPSPI_ClockPhaseSecondEdge;
    sSPI3Config.cpol = kLPSPI_ClockPolarityActiveHigh;
    sSPI3Config.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
    sSPI3Config.whichPcs = kLPSPI_Pcs0;
    sSPI3Config.pinCfg = kLPSPI_SdiInSdoOut;

    LPSPI_MasterInit(LPSPI3, &sSPI3Config, CLOCK_GetRootClockFreq(kCLOCK_Root_Lpspi0304));
    LPSPI_SetFifoWatermarks(LPSPI3, 0, 0);
    LPSPI_FlushFifo(LPSPI3, TRUE, TRUE);
    g_sM1GD3000.sendData = LPSPI3_sendData;
    g_sM1GD3000.resetSetVal = M1_GD3000_reset_setVal;
    g_sM1GD3000.enableSetVal = M1_GD3000_enable_setVal;
}

static uint8_t LPSPI2_sendData(uint8_t ui8Data)
{
    uint32_t ui32Status;

    LPSPI_WriteData(LPSPI2, ui8Data);

    do
    {
        ui32Status = LPSPI_GetStatusFlags(LPSPI2);
    } while ((ui32Status & kLPSPI_TransferCompleteFlag) == 0);

    LPSPI_ClearStatusFlags(LPSPI2, kLPSPI_TransferCompleteFlag);

    do
    {
        ui32Status = LPSPI_GetStatusFlags(LPSPI2);
    } while ((ui32Status & kLPSPI_RxDataReadyFlag) == 0);

    return LPSPI_ReadData(LPSPI2);
}

static void M2_GD3000_reset_setVal(uint8_t ui8Val)
{
    RGPIO_PinWrite(BOARD_INITLPSPI2_M2_GD3000_RESET_GPIO, BOARD_INITLPSPI2_M2_GD3000_RESET_GPIO_PIN, ui8Val);
}

static void M2_GD3000_enable_setVal(uint8_t ui8Val)
{
    RGPIO_PinWrite(BOARD_INITLPSPI2_M2_GD3000_EN_GPIO, BOARD_INITLPSPI2_M2_GD3000_EN_GPIO_PIN, ui8Val);
}
/*!
 * @brief   void M1_InitLPSPI(void)
 *           - Initialization of the SPI3 peripheral for initial M1_GD3000;
 * @param
 *
 * @return
 */
static void M2_InitLPSPI(void)
{

    lpspi_master_config_t sSPI2Config;

    LPSPI_MasterGetDefaultConfig(&sSPI2Config);
    sSPI2Config.baudRate = 4000000;
    sSPI2Config.bitsPerFrame = 8;
    sSPI2Config.cpha = kLPSPI_ClockPhaseSecondEdge;
    sSPI2Config.cpol = kLPSPI_ClockPolarityActiveHigh;
    sSPI2Config.pcsActiveHighOrLow = kLPSPI_PcsActiveLow;
    sSPI2Config.whichPcs = kLPSPI_Pcs0;
    sSPI2Config.pinCfg = kLPSPI_SdiInSdoOut;

    LPSPI_MasterInit(LPSPI2, &sSPI2Config, CLOCK_GetRootClockFreq(kCLOCK_Root_Lpspi0102));
    LPSPI_SetFifoWatermarks(LPSPI2, 0, 0);
    LPSPI_FlushFifo(LPSPI2, TRUE, TRUE);
    g_sM2GD3000.sendData = LPSPI2_sendData;
    g_sM2GD3000.resetSetVal = M2_GD3000_reset_setVal;
    g_sM2GD3000.enableSetVal = M2_GD3000_enable_setVal;
}

static bool_t M1_GetPWMForceFlag(void)
{
	if(XBAR1->CTRL[0] & XBAR_NUM_OUT221_CTRL_STS0_MASK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static bool_t M2_GetPWMForceFlag(void)
{
	if(XBAR1->CTRL[0] & XBAR_NUM_OUT221_CTRL_STS1_MASK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

static bool_t M1_ClearPWMForceFlag(void)
{
	XBAR1->CTRL[0] = (XBAR1->CTRL[0]|XBAR_NUM_OUT221_CTRL_STS0_MASK)&(~XBAR_NUM_OUT221_CTRL_STS1_MASK);
}

static bool_t M2_ClearPWMForceFlag(void)
{
	XBAR1->CTRL[0] = (XBAR1->CTRL[0]|XBAR_NUM_OUT221_CTRL_STS1_MASK)&(~XBAR_NUM_OUT221_CTRL_STS0_MASK);
}
/*!
 * @brief   void InitPWM_M1(void)
 *           - Initialization of the eFlexPWMA peripheral for motor M1
 *           - 3-phase center-aligned PWM
 *           - SM0_trigger0 -> sync motor2, SM1_trigger0 -> ADC,  SM2_trigger0 ->Sinc Filter
 *           - Enable SM0_VAL5  interrupt for M1 abs request interrupt
 *           - Enable SM3_VAL5compare interrupt for motor1 and motor2 slow loop
 * @param   void
 *
 * @return  none
 */
static void M1_InitPWM(void)
{
    int16_t i16Tmp;
    CLOCK_EnableClock(kCLOCK_Pwm4);

    /* PWM base pointer (affects the entire initialization) */
    PWM_Type *PWMBase = (PWM_Type *)PWM4;

    /* Sub-module counter sync and buffered registers reload setting */
    PWMBase->SM[0].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[0].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[0].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0);   // use IPBus clock
    PWMBase->SM[0].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(0); // use local sync for SM0
    PWMBase->SM[0].CTRL2 &= ~PWM_CTRL2_RELOAD_SEL_MASK;                                               // use local reload signal
    PWMBase->SM[0].CTRL2 |= PWM_CTRL2_FORCE_SEL(6)|PWM_CTRL2_FRCEN_MASK;                              // Use external force, initialize counter on force signal

    PWMBase->SM[1].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[1].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[1].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[1].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(2); // use master sync from SM0
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                // use master reload from SM0
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_FORCE_SEL(6)|PWM_CTRL2_FRCEN_MASK;                              // Use external force, initialize counter on force signal

    PWMBase->SM[2].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[2].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[2].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[2].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(2); // use master sync from SM0
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                // use master reload from SM0
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FORCE_SEL(6)|PWM_CTRL2_FRCEN_MASK;                              // Use external force, initialize counter on force signal

    PWMBase->SM[3].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[3].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[3].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[3].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(2); // use master sync from SM0
    PWMBase->SM[3].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                // use master reload from SM0
    PWMBase->SM[3].CTRL2 |= PWM_CTRL2_FORCE_SEL(6)|PWM_CTRL2_FRCEN_MASK;                              // Use external force, initialize counter on force signal

    /* PWM frequency and duty setting */
    PWMBase->SM[0].INIT = -((int16_t)(BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / (2.0 * M1_PWM_FREQ))); // Get INIT value from given PWM frequency
    PWMBase->SM[0].VAL1 = BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / (2.0 * M1_PWM_FREQ) - 1;           // Get VAL1 value from given PWM frequency

    PWMBase->SM[0].VAL0 = 0;
    PWMBase->SM[0].VAL2 = (int16_t)(PWMBase->SM[0].INIT) / 2; // 50% duty
    PWMBase->SM[0].VAL3 = PWMBase->SM[0].VAL1 / 2;            // 50% duty

    PWMBase->SM[1].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[1].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[1].VAL0 = 0;
    PWMBase->SM[1].VAL2 = PWMBase->SM[0].VAL2;
    PWMBase->SM[1].VAL3 = PWMBase->SM[0].VAL3;

    PWMBase->SM[2].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[2].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[2].VAL0 = 0;
    PWMBase->SM[2].VAL2 = PWMBase->SM[0].VAL2;
    PWMBase->SM[2].VAL3 = PWMBase->SM[0].VAL3;

    PWMBase->SM[3].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[3].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[3].VAL0 = 0;

    /* Deadtime setting */
    PWMBase->SM[0].DTCNT0 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[0].DTCNT1 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[1].DTCNT0 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[1].DTCNT1 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[2].DTCNT0 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[2].DTCNT1 = M1_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;

    PWMBase->SM[0].OCTRL = PWM_OCTRL_POLA(1); // PWMA0 low level represents active
    PWMBase->SM[1].OCTRL = PWM_OCTRL_POLA(1); // PWMA1 low level represents active
#if FEATURE_MC_INVERTER_OUTPUT_ENABLE
    PWMBase->SM[2].OCTRL = PWM_OCTRL_POLB(1); // PWMA2 low level represents active
    /* Workaround for PWMA2 & PWMB2 swap: PWMB2 controls top MOS while PWMA2 controls bottom MOS */
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FRCEN_MASK;
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_PWM23_INIT(1) | PWM_CTRL2_PWM45_INIT(0); // PWMA outputs 1 and PWMB outputs 0 at the beginning of a PWM period
//    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FORCE_SEL(5);                            // Use master sync as the force_out for SM2

#else
    PWMBase->SM[2].OCTRL = PWM_OCTRL_POLA(1); // PWMA2 low level represents active
#endif


    /* Trigger for ADC synchronization */
    i16Tmp = (int16_t)(((float_t)M1_ADC_TRIGGER_DELAY - M1_ADC_AVERAGE_TIME * 0.5f) * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000);
    if (i16Tmp >= 0) // Setup ADC triggering point
    {
        PWMBase->SM[1].VAL4 = (int16_t)(PWMBase->SM[0].INIT) + i16Tmp;
    }
    else
    {
        PWMBase->SM[1].VAL4 = (int16_t)(PWMBase->SM[0].VAL1) + i16Tmp;
    }
    PWMBase->SM[1].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // Enable VAL4 for triggering SAR-ADC, SM1_trigger0

    /* Trigger for Sinc Filter synchronization */
//    PWMBase->SM[2].VAL4 = PWMBase->SM[2].VAL1 - (BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT/(2.0*M1_SINC_OUTPUT_CLOCK_FREQ) *M1_SINC_ORDER*M1_SINC_DECIMATION_RATE);
                                                             // from a PWM period starting point: Tpwm - Tdelay = (Tpwm - Tmclk*OSR*Order/2);
                                                             //This trigger time is based on the sinc single conversion mode
//    PWMBase->SM[2].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // Enable VAL4 for triggering SINC filter, SM2_trigger0

    PWMBase->SM[2].VAL4 = PWM_VAL4_VAL4((uint16_t)(1));      // Sinc filter use the continuous mode; When pwm counter reach the VAL4, stop the current conversion, and start a new continuous conversion
    PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(-4124));  // Sinc filter use the continuous mode; When pwm counter reach the VAL5, stop the current conversion, and start a new continuous conversion

    PWMBase->SM[2].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // Enable VAL4 for triggering SINC filter, SM2_trigger0
    PWMBase->SM[2].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 5); // Enable VAL5 for triggering SINC filter, SM2_trigger0

    PWMBase->SM[0].VAL5 = (uint16_t)((int16_t)(PWMBase->SM[0].INIT) + 2000); // Abs request
    PWMBase->SM[3].VAL5 = (uint16_t)((int16_t)(PWMBase->SM[3].VAL0) +50) ;   // Generate the Slow loop ISR for M1 and M2

    PWMBase->SM[0].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 0); // Enable VAL0 for motor2 PWM synchronization, SM0_trigger0

    /* Fault protection setting: Fault protects PWMA&PWMB outputs of SM0~SM2 */
    PWMBase->SM[0].DISMAP[0] = 0;
    PWMBase->SM[1].DISMAP[0] = 0;
    PWMBase->SM[2].DISMAP[0] = 0;
    PWMBase->SM[0].DISMAP[0] = (PWMBase->SM[0].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));
    PWMBase->SM[1].DISMAP[0] = (PWMBase->SM[1].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));
    PWMBase->SM[2].DISMAP[0] = (PWMBase->SM[2].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));

    /* Fault 0,1,2 active in logic level one, manual clearing, safe mode */
    PWMBase->FCTRL = PWM_FCTRL_FLVL((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM)) |
                     PWM_FCTRL_FSAFE((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));

    /* Clear fault flags */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));

    /* PWM fault filter - 5 Fast peripheral clocks sample rate, 5 agreeing
       samples to activate */
    PWMBase->FFILT = PWM_FFILT_FILT_PER(2) | PWM_FFILT_FILT_CNT(0);
    PWMBase->FCTRL2 = PWM_FCTRL2_NOCOMB((1 << M1_FAULT_NUM) | (1 << M1_FAULT_ALT1_NUM) | (1 << M1_FAULT_ALT2_NUM));

    PWMBase->MCTRL |= PWM_MCTRL_LDOK(0xF);

    PWMBase->SM[0].INTEN = (PWMBase->SM[0].INTEN & ~PWM_INTEN_CMPIE_MASK) | PWM_INTEN_CMPIE(1<<5);
    EnableIRQ(PWM4_0_IRQn);
    NVIC_SetPriority(PWM4_0_IRQn, 3U);

    PWMBase->SM[3].INTEN = (PWMBase->SM[3].INTEN & ~PWM_INTEN_CMPIE_MASK) | PWM_INTEN_CMPIE(1<<5);
    EnableIRQ(PWM4_3_IRQn);             // Enable the PWM_SM3_CMP_ISR for MC slow loop
    NVIC_SetPriority(PWM4_3_IRQn, 4U);


    /* Initialize MC driver */
    g_sM1Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;

    g_sM1Pwm3ph.ui16PhASubNum = 0U; /* PWMA phase A sub-module number */
    g_sM1Pwm3ph.ui16PhBSubNum = 1U; /* PWMA phase B sub-module number */
    g_sM1Pwm3ph.ui16PhCSubNum = 2U; /* PWMA phase C sub-module number */

    g_sM1Pwm3ph.ui16FaultFixNum = M1_FAULT_NUM;       /* PWMA fixed-value over-current fault number */
    g_sM1Pwm3ph.ui16FaultAdjNum = M1_FAULT_ALT1_NUM;  /* PWMA adjustable over-current fault number */
    g_sM1Pwm3ph.ui16FaultAdjNum1 = M1_FAULT_ALT2_NUM; /* PWMA adjustable over-current fault number */
}

/*!
 * @brief   void InitPWM_M2(void)
 *           - Initialization of the eFlexPWMA peripheral for motor M2
 *           - 3-phase center-aligned PWM
 *           - SM1_trigger0 -> ADC
 *           - Enable SM0_VAL5  interrupt for M1 abs request interrupt
 *           - Enable SM3_VAL2/3 compare interrupt for motor1 and motor2 fast loop
 *
 * @param   void
 *
 * @return  none
 */
static void M2_InitPWM(void)
{
    int16_t i16Tmp;
    CLOCK_EnableClock(kCLOCK_Pwm2);

    /* PWM base pointer (affects the entire initialization) */
    PWM_Type *PWMBase = (PWM_Type *)PWM2;

    /* Sub-module counter sync and buffered registers reload setting */
    PWMBase->SM[0].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[0].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[0].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(0);   // use IPBus clock
    PWMBase->SM[0].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[0].CTRL2 = (PWMBase->SM[0].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(3); // use external sync
    PWMBase->SM[0].CTRL2 &= ~PWM_CTRL2_RELOAD_SEL_MASK;                                               // use local reload signal

    PWMBase->SM[1].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[1].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[1].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[1].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[1].CTRL2 = (PWMBase->SM[1].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(3); // use external sync
    PWMBase->SM[1].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                // use master reload from SM0

    PWMBase->SM[2].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[2].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[2].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[2].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[2].CTRL2 = (PWMBase->SM[2].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(3); // use external sync
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                // use master reload from SM0

    PWMBase->SM[3].CTRL |= PWM_CTRL_HALF_MASK | PWM_CTRL_FULL_MASK;                                   // half cycle reload
    PWMBase->SM[3].CTRL &= ~PWM_CTRL_COMPMODE_MASK;                                                   // Edge is generated on counter "equal to" value register
    PWMBase->SM[3].CTRL &= ~PWM_CTRL_LDMOD_MASK;                                                      // buffered registers take effect at PWM reload signal when LDOK is set
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_CLK_SEL_MASK) | PWM_CTRL2_CLK_SEL(2);   // SM0's clock is used
    PWMBase->SM[3].CTRL2 &= ~PWM_CTRL2_INDEP_MASK;                                                    // complementary mode
    PWMBase->SM[3].CTRL2 = (PWMBase->SM[3].CTRL2 & ~PWM_CTRL2_INIT_SEL_MASK) | PWM_CTRL2_INIT_SEL(3); // use local sync from SM0
    PWMBase->SM[3].CTRL2 |= PWM_CTRL2_RELOAD_SEL_MASK;                                                  // use master reload from SM0

    /* PWM frequency and duty setting */
    PWMBase->SM[0].INIT = -((int16_t)(BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / (2.0 * M2_PWM_FREQ))); // Get INIT value from given PWM frequency
    PWMBase->SM[0].VAL1 = BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / (2.0 * M2_PWM_FREQ) - 1;           // Get VAL1 value from given PWM frequency

    PWMBase->SM[0].VAL0 = 0;
    PWMBase->SM[0].VAL2 = (int16_t)(PWMBase->SM[0].INIT) / 2; // 50% duty
    PWMBase->SM[0].VAL3 = PWMBase->SM[0].VAL1 / 2;            // 50% duty

    PWMBase->SM[1].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[1].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[1].VAL0 = 0;
    PWMBase->SM[1].VAL2 = PWMBase->SM[0].VAL2;
    PWMBase->SM[1].VAL3 = PWMBase->SM[0].VAL3;

    PWMBase->SM[2].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[2].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[2].VAL0 = 0;
    PWMBase->SM[2].VAL2 = PWMBase->SM[0].VAL2;
    PWMBase->SM[2].VAL3 = PWMBase->SM[0].VAL3;

    PWMBase->SM[3].INIT = PWMBase->SM[0].INIT;
    PWMBase->SM[3].VAL1 = PWMBase->SM[0].VAL1;
    PWMBase->SM[3].VAL0 = 0;
    PWMBase->SM[3].VAL2 = -900;                               // trigger fast loop interrupt 4us before PWM register half-cycle update.
    PWMBase->SM[3].VAL3 = PWMBase->SM[0].VAL1 - 900;          // trigger fast loop interrupt 4us before PWM register full-cycle update.

    /* Deadtime setting */
    PWMBase->SM[0].DTCNT0 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[0].DTCNT1 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[1].DTCNT0 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[1].DTCNT1 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[2].DTCNT0 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;
    PWMBase->SM[2].DTCNT1 = M2_PWM_DEADTIME * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000;

    PWMBase->SM[0].OCTRL = PWM_OCTRL_POLA(1); // PWMA0 low level represents active
    PWMBase->SM[1].OCTRL = PWM_OCTRL_POLA(1); // PWMA1 low level represents active
#if FEATURE_MC_INVERTER_OUTPUT_ENABLE
    PWMBase->SM[2].OCTRL = PWM_OCTRL_POLB(1); // PWMB2 low level represents active
    /* Workaround for PWMA2 & PWMB2 swap: PWMB2 controls top MOS while PWMA2 controls bottom MOS */
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FRCEN_MASK;
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_PWM23_INIT(1) | PWM_CTRL2_PWM45_INIT(0); // PWMA outputs 1 and PWMB outputs 0 at the beginning of a PWM period
                                                                               //    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FORCE_MASK; // Force output initial values
    PWMBase->SM[2].CTRL2 |= PWM_CTRL2_FORCE_SEL(5);                            // Use master sync as the force_out for SM2
#else
    PWMBase->SM[2].OCTRL = PWM_OCTRL_POLA(1); // PWMB2 low level represents active
#endif
    /* Trigger for ADC synchronization */
    i16Tmp = (int16_t)(((float_t)M2_ADC_TRIGGER_DELAY - M2_ADC_AVERAGE_TIME * 0.5f) * BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT / 1000000);
    if (i16Tmp >= 0) // Setup ADC triggering point
    {
        PWMBase->SM[1].VAL4 = (int16_t)(PWMBase->SM[0].INIT) + i16Tmp;
    }
    else
    {
        PWMBase->SM[1].VAL4 = (int16_t)(PWMBase->SM[0].VAL1) + i16Tmp;
    }
    PWMBase->SM[1].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // Enable VAL4 for triggering ADC, SM1_trigger0

    /* Trigger for Sinc Filter synchronization  -Use the PWM4's Trigger*/
//    PWMBase->SM[2].VAL4 = PWMBase->SM[2].VAL1 - (BOARD_BOOTCLOCKRUN_BUS_WAKEUP_CLK_ROOT/(2.0*M2_SINC_OUTPUT_CLOCK_FREQ) *M2_SINC_ORDER*M2_SINC_DECIMATION_RATE);
//     from a PWM period starting point: Tpwm - Tdelay = (Tpwm - Tmclk*OSR*Order/2)
//    PWMBase->SM[2].TCTRL |= PWM_TCTRL_OUT_TRIG_EN(1 << 4); // Enable VAL4 for triggering SINC filter, SM2_trigger0

    PWMBase->SM[0].VAL5 = (uint16_t)((int16_t)(PWMBase->SM[0].INIT) + 2000);//ABS request
    PWMBase->SM[1].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));
    PWMBase->SM[2].VAL5 = PWM_VAL5_VAL5((uint16_t)(0));

    /* Fault protection setting: Fault protects PWMA&PWMB outputs of SM0~SM2 */
    PWMBase->SM[0].DISMAP[0] = 0;
    PWMBase->SM[1].DISMAP[0] = 0;
    PWMBase->SM[2].DISMAP[0] = 0;
    PWMBase->SM[0].DISMAP[0] = (PWMBase->SM[0].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));
    PWMBase->SM[1].DISMAP[0] = (PWMBase->SM[1].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));
    PWMBase->SM[2].DISMAP[0] = (PWMBase->SM[2].DISMAP[0] & ~(PWM_DISMAP_DIS0A_MASK | PWM_DISMAP_DIS0B_MASK)) | PWM_DISMAP_DIS0A((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM)) |
                               PWM_DISMAP_DIS0B((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));

    /* Fault 0,1,2 active in logic level one, manual clearing, safe mode */
    PWMBase->FCTRL = PWM_FCTRL_FLVL((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM)) |
                     PWM_FCTRL_FSAFE((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));

    /* Clear fault flags */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFLAG_MASK) | PWM_FSTS_FFLAG(0xF);

    /* PWMs are re-enabled at PWM full cycle */
    PWMBase->FSTS = (PWMBase->FSTS & ~PWM_FSTS_FFULL_MASK) | PWM_FSTS_FFULL((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));

    /* PWM fault filter - 5 Fast peripheral clocks sample rate, 5 agreeing
       samples to activate */
    PWMBase->FFILT = PWM_FFILT_FILT_PER(2) | PWM_FFILT_FILT_CNT(0);
    PWMBase->FCTRL2 = PWM_FCTRL2_NOCOMB((1 << M2_FAULT_NUM) | (1 << M2_FAULT_ALT1_NUM) | (1 << M2_FAULT_ALT2_NUM));

    PWMBase->MCTRL |= PWM_MCTRL_LDOK(0xF);

    PWMBase->SM[0].INTEN = PWM_INTEN_CMPIE(0x20); // Enable Val5 compare

    EnableIRQ(PWM2_0_IRQn);
    NVIC_SetPriority(PWM2_0_IRQn, 3U);

    PWMBase->SM[3].INTEN |= (PWMBase->SM[3].INTEN & ~PWM_INTEN_CMPIE_MASK) | PWM_INTEN_CMPIE(1<<2)| PWM_INTEN_CMPIE(1<<3);
    EnableIRQ(PWM2_3_IRQn);                    //Enable the COMP3_ISR for MC fast loop
    NVIC_SetPriority(PWM2_3_IRQn, 0U);

    /* Initialize MC driver */
    g_sM2Pwm3ph.pui32PwmBaseAddress = (PWM_Type *)PWMBase;

    g_sM2Pwm3ph.ui16PhASubNum = 0U; /* PWMA phase A sub-module number */
    g_sM2Pwm3ph.ui16PhBSubNum = 1U; /* PWMA phase B sub-module number */
    g_sM2Pwm3ph.ui16PhCSubNum = 2U; /* PWMA phase C sub-module number */

    g_sM2Pwm3ph.ui16FaultFixNum = M2_FAULT_NUM;       /* PWMA fixed-value over-current fault number */
    g_sM2Pwm3ph.ui16FaultAdjNum = M2_FAULT_ALT1_NUM;  /* PWMA adjustable over-current fault number */
    g_sM2Pwm3ph.ui16FaultAdjNum1 = M2_FAULT_ALT2_NUM; /* PWMA adjustable over-current fault number */
}
/*!
 * @brief   void M1_InitQdc(void)
 *           - Initialization of the eQDC peripheral for motor M1 index encoder
 *
 * @param   void
 *
 * @return  none
 */
static void M1_InitQdc(void)
{
    CLOCK_EnableClock(kCLOCK_Enc1);

    EQDC_Type *QDCBase = (EQDC_Type *)EQDC1;

    QDCBase->CTRL2 &= ~EQDC_CTRL2_LDMOD_MASK;
    QDCBase->CTRL &= ~EQDC_CTRL_LDOK_MASK;

    QDCBase->LMOD = 4 * M1_ENCODER_LINES - 1;
    QDCBase->UMOD = 0;
    QDCBase->LPOS = 0;
    QDCBase->UPOS = 0;

    QDCBase->CTRL |= EQDC_CTRL_LDOK_MASK;
    while (QDCBase->CTRL & EQDC_CTRL_LDOK_MASK)
        ;

    QDCBase->FILT = EQDC_FILT_FILT_CNT(2) | EQDC_FILT_FILT_PER(1) | EQDC_FILT_PRSC(M1_QDC_TIMER_PRESCALER);
    QDCBase->CTRL2 = EQDC_CTRL2_REVMOD_MASK | EQDC_CTRL2_PMEN_MASK; // REV is controlled by modulo counting

	g_sM1QdcSensor.pQDC_base = QDCBase;
    /* Position and revolution generation init */
	g_sM1QdcSensor.ui16Line = M1_ENCODER_LINES;
	g_sM1QdcSensor.ui16PolePair = M1_MOTOR_PP;
	g_sM1QdcSensor.f32ReciprocalLines = M1_QDC_LINE_RECIPROCAL_4_REV_FRAC_GEN;
	g_sM1QdcSensor.i32Q10Cnt2PosGain = M1_QDC_LINE_RECIPROCAL_4_POS_GEN; 
	MCDRV_QdcInit(&g_sM1QdcSensor);
    /* Speed calculation (based on QDC HW) init */
    g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B0 = M1_QDC_SPEED_FILTER_IIR_B0_FRAC;
	g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B1 = M1_QDC_SPEED_FILTER_IIR_B1_FRAC;
	g_sM1QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32A1 = M1_QDC_SPEED_FILTER_IIR_A1_FRAC;
	g_sM1QdcSensor.sSpeed.f32SpeedCalConst = M1_SPEED_CAL_CONST;
	g_sM1QdcSensor.sSpeed.fltSpeedFrac16ToAngularCoeff = M1_SPEED_FRAC_TO_ANGULAR_COEFF;
	MCDRV_QdcSpeedCalInit(&g_sM1QdcSensor);
    /* Speed calculation (based on Tracking Observer) init */
    g_sM1QdcSensor.sSpeedEstim.sTO.fltPGain = M1_QDC_TO_KP_GAIN;
	g_sM1QdcSensor.sSpeedEstim.sTO.fltIGain = M1_QDC_TO_KI_GAIN;
	g_sM1QdcSensor.sSpeedEstim.sTO.fltThGain = M1_QDC_TO_THETA_GAIN;
	MCDRV_QdcToSpeedCalInit(&g_sM1QdcSensor);
}
/*!
 * @brief   void M1_InitQdc(void)
 *           - Initialization of the eQDC peripheral for motor M2 index encoder
 *
 * @param   void
 *
 * @return  none
 */
static void M2_InitQdc(void)
{
    CLOCK_EnableClock(kCLOCK_Enc2);

    EQDC_Type *QDCBase = (EQDC_Type *)EQDC2;

    QDCBase->CTRL2 &= ~EQDC_CTRL2_LDMOD_MASK;
    QDCBase->CTRL &= ~EQDC_CTRL_LDOK_MASK;

    QDCBase->LMOD = 4 * M2_ENCODER_LINES - 1;
    QDCBase->UMOD = 0;
    QDCBase->LPOS = 0;
    QDCBase->UPOS = 0;

    QDCBase->CTRL |= EQDC_CTRL_LDOK_MASK;
    while (QDCBase->CTRL & EQDC_CTRL_LDOK_MASK);

    QDCBase->FILT = EQDC_FILT_FILT_CNT(2) | EQDC_FILT_FILT_PER(1) | EQDC_FILT_PRSC(M1_QDC_TIMER_PRESCALER);
    QDCBase->CTRL2 = EQDC_CTRL2_REVMOD_MASK | EQDC_CTRL2_PMEN_MASK; // REV is controlled by modulo counting

	g_sM2QdcSensor.pQDC_base = QDCBase;
    /* Position and revolution generation init */
	g_sM2QdcSensor.ui16Line = M2_ENCODER_LINES;
	g_sM2QdcSensor.ui16PolePair = M2_MOTOR_PP;
	g_sM2QdcSensor.f32ReciprocalLines = M2_QDC_LINE_RECIPROCAL_4_REV_FRAC_GEN;
	g_sM2QdcSensor.i32Q10Cnt2PosGain = M2_QDC_LINE_RECIPROCAL_4_POS_GEN; 
	MCDRV_QdcInit(&g_sM2QdcSensor);
    /* Speed calculation (based on QDC HW) init */
    g_sM2QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B0 = M2_QDC_SPEED_FILTER_IIR_B0_FRAC;
	g_sM2QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32B1 = M2_QDC_SPEED_FILTER_IIR_B1_FRAC;
	g_sM2QdcSensor.sSpeed.sQDCSpeedFilter.sFltCoeff.f32A1 = M2_QDC_SPEED_FILTER_IIR_A1_FRAC;
	g_sM2QdcSensor.sSpeed.f32SpeedCalConst = M2_SPEED_CAL_CONST;
	g_sM2QdcSensor.sSpeed.fltSpeedFrac16ToAngularCoeff = M2_SPEED_FRAC_TO_ANGULAR_COEFF;
	MCDRV_QdcSpeedCalInit(&g_sM2QdcSensor);
    /* Speed calculation (based on Tracking Observer) init */
    g_sM2QdcSensor.sSpeedEstim.sTO.fltPGain = M2_QDC_TO_KP_GAIN;
	g_sM2QdcSensor.sSpeedEstim.sTO.fltIGain = M2_QDC_TO_KI_GAIN;
	g_sM2QdcSensor.sSpeedEstim.sTO.fltThGain = M2_QDC_TO_THETA_GAIN;
	MCDRV_QdcToSpeedCalInit(&g_sM2QdcSensor);
}
/*!
 * @brief   void M1_InitSincFilter(void)
 *           - Initialization of the Sinc Filter peripheral for Sigma-Delta ADC sample
 *
 * @param   void
 *
 * @return  none
 */
static void M1_InitSincFilter(void)
{
    bool_t bStatusPass;
    CLOCK_EnableClock(kCLOCK_Sinc1);

    SINC_Type *SINCBase = (SINC_Type *)SINC1;
    g_sM1SincFilter.psSincBaseAddress = SINC1;

    g_sM1SincFilter.ui8ClkDiv = M1_SINC_MODULE_CLOCK_DIV;
    g_sM1SincFilter.ui8ClkPrescaler = M1_SINC_MODULE_CLOCK_PRESCALE;
    g_sM1SincFilter.ui8FifoDepth = (uint8_t)M1_SINC_FIFO_DEPTH;
    g_sM1SincFilter.dblFifoDepth = M1_SINC_FIFO_DEPTH;
    g_sM1SincFilter.ui8Order = M1_SINC_ORDER;
    g_sM1SincFilter.ui8IaChnNum = M1_SINC_IA_CHN;
    g_sM1SincFilter.ui8IbChnNum = M1_SINC_IB_CHN;
    g_sM1SincFilter.ui8IcChnNum = M1_SINC_IC_CHN;
    g_sM1SincFilter.ui8UdcChnNum = M1_SINC_UDC_CHN;
    g_sM1SincFilter.fltIScale = M1_SINC_I_SCALE;
    g_sM1SincFilter.fltUScale = M1_SINC_U_SCALE;
    g_sM1SincFilter.fltRsltScale = M1_SINC_RSLT_SCALE;
    g_sM1SincFilter.ui8RsltRightShiftBits = M1_SINC_RSLT_RSHIFT_BITS;
    g_sM1SincFilter.ui16DecimationRate = M1_SINC_DECIMATION_RATE;
    g_sM1SincFilter.dblDecimationClockFreq = M1_SINC_DECIMATION_CLOCK_FREQ;
    g_sM1SincFilter.dblSincOutputClockFreq = M1_SINC_OUTPUT_CLOCK_FREQ;

    SINCBase->MCR = SINC_MCR_RST_MASK;
    SINCBase->MCR &= ~SINC_MCR_RST_MASK; // Reset SINC module

    bStatusPass = MCDRV_SincFilterInit(&g_sM1SincFilter);
//    while(bStatusPass == FALSE); // Debug. It's stuck here when the ratio between decimation frequency and PWM frequency is not an integer, only used the FIFO function need use this part

    SINCBase->CHANNEL[g_sM1SincFilter.ui8IaChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(1)|SINC_CCFR_ICSEL(0)|SINC_CCFR_ICESEL(2); // Use external data bit, MCLK0_OUT, falling edge, signed result, no shift
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IbChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(3)|SINC_CCFR_ICSEL(0)|SINC_CCFR_ICESEL(2); // Use the trigger from previous channel
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IcChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(3)|SINC_CCFR_ICSEL(0)|SINC_CCFR_ICESEL(2); // Use the trigger from previous channel
    SINCBase->CHANNEL[g_sM1SincFilter.ui8UdcChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(3)|SINC_CCFR_ICSEL(0)|SINC_CCFR_ICESEL(2); // Use the trigger from previous channel

    SINCBase->CHANNEL[g_sM1SincFilter.ui8IaChnNum].CACFR = SINC_CACFR_IBDLY(0);
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IbChnNum].CACFR = SINC_CACFR_IBDLY(0);
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IcChnNum].CACFR = SINC_CACFR_IBDLY(0);
    SINCBase->CHANNEL[g_sM1SincFilter.ui8UdcChnNum].CACFR = SINC_CACFR_IBDLY(0);

    SINCBase->CHANNEL[g_sM1SincFilter.ui8IaChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IbChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    SINCBase->CHANNEL[g_sM1SincFilter.ui8IcChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    
//    SINCBase->NIE = SINC_NIE_COCIE0_MASK; // Enable conversion completion interrupt of channel 0
//
//    EnableIRQ(SINC1_CH0_IRQn);

#if SINC_CLOSEDLOOP == ENABLED    
//    NVIC_SetPriority(SINC1_CH0_IRQn, 1U);
#else
    NVIC_SetPriority(SINC1_CH0_IRQn, 4U);
#endif
}

static void M2_InitSincFilter(void)
{
    bool_t bStatusPass;
    CLOCK_EnableClock(kCLOCK_Sinc2);

    SINC_Type *SINCBase = (SINC_Type *)SINC2;
    g_sM2SincFilter.psSincBaseAddress = SINC2;

    g_sM2SincFilter.ui8ClkDiv = M2_SINC_MODULE_CLOCK_DIV;
    g_sM2SincFilter.ui8ClkPrescaler = M2_SINC_MODULE_CLOCK_PRESCALE;
    g_sM2SincFilter.ui8FifoDepth = (uint8_t)M2_SINC_FIFO_DEPTH;
    g_sM2SincFilter.dblFifoDepth = M2_SINC_FIFO_DEPTH;
    g_sM2SincFilter.ui8Order = M2_SINC_ORDER;
    g_sM2SincFilter.ui8IaChnNum = M2_SINC_IA_CHN;
    g_sM2SincFilter.ui8IbChnNum = M2_SINC_IB_CHN;
    g_sM2SincFilter.ui8IcChnNum = M2_SINC_IC_CHN;
    g_sM2SincFilter.ui8UdcChnNum = M2_SINC_UDC_CHN;
    g_sM2SincFilter.fltIScale = M2_SINC_I_SCALE;
    g_sM2SincFilter.fltUScale = M2_SINC_U_SCALE;
    g_sM2SincFilter.fltRsltScale = M2_SINC_RSLT_SCALE;
    g_sM2SincFilter.ui8RsltRightShiftBits = M2_SINC_RSLT_RSHIFT_BITS;
    g_sM2SincFilter.ui16DecimationRate = M2_SINC_DECIMATION_RATE;
    g_sM2SincFilter.dblDecimationClockFreq = M2_SINC_DECIMATION_CLOCK_FREQ;
    g_sM2SincFilter.dblSincOutputClockFreq = M2_SINC_OUTPUT_CLOCK_FREQ;

    SINCBase->MCR = SINC_MCR_RST_MASK;
    SINCBase->MCR &= ~SINC_MCR_RST_MASK; // Reset SINC module
    bStatusPass = MCDRV_SincFilterInit(&g_sM2SincFilter);
    g_sM2SincFilter.psSincBaseAddress->CHANNEL[g_sM2SincFilter.ui8UdcChnNum].CCR = 0; // Disable Udc channel
//    while(bStatusPass == FALSE); // Debug. It's stuck here when the ratio between decimation frequency and PWM frequency is not an integer, only used the FIFO function need use this part


    SINCBase->CHANNEL[g_sM2SincFilter.ui8IaChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(1)|SINC_CCFR_ICSEL(3)|SINC_CCFR_ICESEL(2); // Use external data bit, MCLK0_OUT, falling edge, signed result, no shift
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IbChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(3)|SINC_CCFR_ICSEL(3)|SINC_CCFR_ICESEL(2); // Use the trigger from previous channel
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IcChnNum].CCFR = SINC_CCFR_IBSEL(0)|SINC_CCFR_ITSEL(3)|SINC_CCFR_ICSEL(3)|SINC_CCFR_ICESEL(2); // Use the trigger from previous channel

    SINCBase->CHANNEL[g_sM2SincFilter.ui8IaChnNum].CACFR = SINC_CACFR_IBDLY(0);
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IbChnNum].CACFR = SINC_CACFR_IBDLY(0);
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IcChnNum].CACFR = SINC_CACFR_IBDLY(0);

    SINCBase->CHANNEL[g_sM2SincFilter.ui8IaChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IbChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    SINCBase->CHANNEL[g_sM2SincFilter.ui8IcChnNum].CPROT = SINC_CPROT_SCDBK_MASK|SINC_CPROT_SCDOP(0)|SINC_CPROT_SCDCM(0)|SINC_CPROT_SCDLMT(5); // Both repeating 0 and 1 triggers protection.
    
    SINCBase->NIE = SINC_NIE_COCIE1_MASK; // Enable conversion completion interrupt of channel 1

//    EnableIRQ(SINC2_CH1_IRQn);
#if SINC_CLOSEDLOOP == ENABLED    
//    NVIC_SetPriority(SINC2_CH1_IRQn, 1U);
#else
    NVIC_SetPriority(SINC2_CH1_IRQn, 4U);
#endif
}

static void qtimer1_1_init(void)
{
    CLOCK_EnableClock(kCLOCK_Qtimer1);

    TMR1->CHANNEL[1].CTRL = TMR_CTRL_PCS(0x8); // IP bus clock
}

static void qtimer1_2_init(void)
{
    CLOCK_EnableClock(kCLOCK_Qtimer1);

    TMR1->CHANNEL[2].CTRL = TMR_CTRL_PCS(0x8); // IP bus clock
}

static void qtimer1_3_init(void)
{
    CLOCK_EnableClock(kCLOCK_Qtimer1);

    TMR1->CHANNEL[3].CTRL = TMR_CTRL_PCS(0x8); /* IP bus clock */
}

RAM_FUNC_CRITICAL static void M1_tamagawa_transmit_enable(void)
{
    RGPIO_PinWrite(BOARD_UART5_M1_485_GPIO, BOARD_UART5_M1_485_GPIO_PIN, 1u);
}
RAM_FUNC_CRITICAL static void M1_tamagawa_receive_enable(void)
{
    RGPIO_PinWrite(BOARD_UART5_M1_485_GPIO, BOARD_UART5_M1_485_GPIO_PIN, 0u);
}
RAM_FUNC_CRITICAL static void M2_tamagawa_transmit_enable(void)
{
    RGPIO_PinWrite(BOARD_UART8_M2_485_GPIO, BOARD_UART8_M2_485_GPIO_PIN, 1u);
}
RAM_FUNC_CRITICAL static void M2_tamagawa_receive_enable(void)
{
    RGPIO_PinWrite(BOARD_UART8_M2_485_GPIO, BOARD_UART8_M2_485_GPIO_PIN, 0u);
}
/*!
 * @brief   void M1_InitTamagawaAbs(void)
 *           - Initialization of the uart and DMA peripheral for motor M1 abs encoder
 *
 * @param   void
 *
 * @return  none
 */
static void M1_InitTamagawaAbs(void)
{
    g_sM1TamagawaAbs.ui8DMAChannel = M1_TAMAGAWA_REC_DMA_CHANNEL; // Use channel 0 of DMA4 for receiving response frame
    g_sM1TamagawaAbs.LPUART = LPUART5;  // Specify the uart for the physical layer communication
    g_sM1TamagawaAbs.ui8OneTurnBitNum = M1_TAMAGAWA_POSITION_CNT_RESOLUTION;
    g_sM1TamagawaAbs.ui8PolePair = M1_MOTOR_PP;
    g_sM1TamagawaAbs.sSpeed.fltSpeedElecRadScale = M1_RAD_MAX;
    g_sM1TamagawaAbs.sSpeed.i32Q23SpeedCalConst = M1_TAMAGAWA_SPEED_CAL_CONST;
    g_sM1TamagawaAbs.pTransmitEnable = M1_tamagawa_transmit_enable;
    g_sM1TamagawaAbs.pReceiveEnable = M1_tamagawa_receive_enable;
    g_sM1TamagawaAbs.bPosAbsoluteFlag = M1_TAMAGAWA_IF_USE_ABS_POSITION;
    g_sM1TamagawaAbs.f32PosMechOffset = M1_TAMAGAWA_ABS_POSITION_COMP;
//    g_sM1TamagawaAbs.pGetPWMForceFlag = M1_GetPWMForceFlag;
//    g_sM1TamagawaAbs.pClearPWMForceFlag = M1_ClearPWMForceFlag;

    /* Speed calculation (based on Tracking Observer) init */
    g_sM1TamagawaAbs.sSpeedEstim.sTO.fltPGain = M1_ABS_TO_KP_GAIN;
    g_sM1TamagawaAbs.sSpeedEstim.sTO.fltIGain = M1_ABS_TO_KI_GAIN;
    g_sM1TamagawaAbs.sSpeedEstim.sTO.fltThGain = M1_ABS_TO_THETA_GAIN;
	MCDRV_AbsToSpeedCalInit(&g_sM1TamagawaAbs);

    /* Initialize UART */
    lpuart_config_t lpuartConfig;
    /*
     * lpuartConfig.parityMode = kLPUART_ParityDisabled;
     * lpuartConfig.stopBitCount = kLPUART_OneStopBit;
     * lpuartConfig.txFifoWatermark = 0;
     * lpuartConfig.rxFifoWatermark = 0;
     */
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = M1_TAMAGAWA_COM_BAUDRATE;
    lpuartConfig.enableTx = true;
    lpuartConfig.enableRx = false;
    LPUART_Init(LPUART5, &lpuartConfig, CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart0506));
    LPUART5->BAUD |= LPUART_BAUD_RDMAE_MASK; // Enable receive DMA
    EnableIRQ(LPUART5_IRQn);
    IRQ_SetPriority(LPUART5_IRQn, 0);

    /* Initialize DMA */
    edma_config_t config;
    edma_channel_config_t channelConfig = {
        .enableMasterIDReplication = true,
        .securityLevel = kEDMA_ChannelSecurityLevelSecure,
        .protectionLevel = kEDMA_ChannelProtectionLevelPrivileged,
    };
    EDMA_GetDefaultConfig(&config);
    config.enableMasterIdReplication = true;
    config.channelConfig[0] = &channelConfig;
    EDMA_Init(DMA4, &config);
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].CH_MUX |= kDma4RequestMuxLPUART5Rx;
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].SADDR = (uint32_t)(&LPUART5->DATA); /*source address*/
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].SOFF = 0;
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].ATTR = 0x0000;       /*transfer data size is 8-bit*/
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].NBYTES_MLOFFYES = 1; /* transfer 1 bytes each minor loop*/
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].SLAST_SDA = 0;       /* final increment to source */
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].DOFF = 1;           /* increment destination address */
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].CSR = DMA4_CSR_INTMAJOR_MASK; /*enable major loop interrupt. Major loop length is configured before transmitting any data through UART */
    DMA4->TCD[M1_TAMAGAWA_REC_DMA_CHANNEL].CH_CSR |= DMA4_CH_CSR_ERQ_MASK;
    EnableIRQ(DMA4_CH0_CH1_CH32_CH33_IRQn);
    IRQ_SetPriority(DMA4_CH0_CH1_CH32_CH33_IRQn, 1);

     MCDRV_TamagawaAbsInit(&g_sM1TamagawaAbs); // Must be called after DMA initialization
}


static void M2_InitTamagawaAbs(void)
{
    g_sM2TamagawaAbs.ui8DMAChannel = M2_TAMAGAWA_REC_DMA_CHANNEL; // Use channel 2 of DMA4 for receiving response frame
    g_sM2TamagawaAbs.LPUART = LPUART8;  // Specify the uart for the physical layer communication
    g_sM2TamagawaAbs.ui8OneTurnBitNum = M2_TAMAGAWA_POSITION_CNT_RESOLUTION;
    g_sM2TamagawaAbs.ui8PolePair = M2_MOTOR_PP;
    g_sM2TamagawaAbs.sSpeed.fltSpeedElecRadScale = M2_RAD_MAX;
    g_sM2TamagawaAbs.sSpeed.i32Q23SpeedCalConst = M2_TAMAGAWA_SPEED_CAL_CONST;
    g_sM2TamagawaAbs.pTransmitEnable = M2_tamagawa_transmit_enable;
    g_sM2TamagawaAbs.pReceiveEnable = M2_tamagawa_receive_enable;
    g_sM2TamagawaAbs.bPosAbsoluteFlag = M2_TAMAGAWA_IF_USE_ABS_POSITION;
    g_sM2TamagawaAbs.f32PosMechOffset = M2_TAMAGAWA_ABS_POSITION_COMP;
//    g_sM2TamagawaAbs.pGetPWMForceFlag = M2_GetPWMForceFlag;
//    g_sM2TamagawaAbs.pClearPWMForceFlag = M2_ClearPWMForceFlag;

    /* Speed calculation (based on Tracking Observer) init */
    g_sM2TamagawaAbs.sSpeedEstim.sTO.fltPGain = M2_ABS_TO_KP_GAIN;
    g_sM2TamagawaAbs.sSpeedEstim.sTO.fltIGain = M2_ABS_TO_KI_GAIN;
    g_sM2TamagawaAbs.sSpeedEstim.sTO.fltThGain = M2_ABS_TO_THETA_GAIN;
	MCDRV_AbsToSpeedCalInit(&g_sM2TamagawaAbs);

    /* Initialize UART */
    lpuart_config_t lpuartConfig;
    /*
     * lpuartConfig.parityMode = kLPUART_ParityDisabled;
     * lpuartConfig.stopBitCount = kLPUART_OneStopBit;
     * lpuartConfig.txFifoWatermark = 0;
     * lpuartConfig.rxFifoWatermark = 0;
     */
    LPUART_GetDefaultConfig(&lpuartConfig);
    lpuartConfig.baudRate_Bps = M2_TAMAGAWA_COM_BAUDRATE;
    lpuartConfig.enableTx = true;
    lpuartConfig.enableRx = false;
    LPUART_Init(LPUART8, &lpuartConfig, CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart0708));
    LPUART8->BAUD |= LPUART_BAUD_RDMAE_MASK; // Enable receive DMA
    EnableIRQ(LPUART8_IRQn);
    IRQ_SetPriority(LPUART8_IRQn, 0);

    /* Initialize DMA */
    edma_config_t config;
    edma_channel_config_t channelConfig = {
        .enableMasterIDReplication = true,
        .securityLevel = kEDMA_ChannelSecurityLevelSecure,
        .protectionLevel = kEDMA_ChannelProtectionLevelPrivileged,
    };
    EDMA_GetDefaultConfig(&config);
    config.enableMasterIdReplication = true;
    config.channelConfig[2] = &channelConfig;
    EDMA_Init(DMA4, &config);
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].CH_MUX |= kDma4RequestMuxLPUART8Rx;
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].SADDR = (uint32_t)(&LPUART8->DATA); /*source address*/
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].SOFF = 0;
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].ATTR = 0x0000;       /*transfer data size is 8-bit*/
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].NBYTES_MLOFFYES = 1; /* transfer 1 bytes each minor loop*/
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].SLAST_SDA = 0;       /* final increment to source */
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].DOFF = 1;           /* increment destination address */
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].CSR = DMA4_CSR_INTMAJOR_MASK; /*enable major loop interrupt. Major loop length is configured before transmitting any data through UART */
    DMA4->TCD[M2_TAMAGAWA_REC_DMA_CHANNEL].CH_CSR |= DMA4_CH_CSR_ERQ_MASK;
    EnableIRQ(DMA4_CH2_CH3_CH34_CH35_IRQn);
    IRQ_SetPriority(DMA4_CH2_CH3_CH34_CH35_IRQn, 1);

    MCDRV_TamagawaAbsInit(&g_sM2TamagawaAbs); // Must be called after DMA initialization
}


/*!
 * @brief   void peripherals_manual_init(void)
 *           - Motor control driver main initialization
 *           - Calls initialization functions of peripherals required for motor
 *             control functionality
 *
 * @param   void
 *
 * @return  none
 */
void peripherals_manual_init(void)
{
    M1_M2_InitADC();
    M1_InitQtimer1_0();
    M2_InitQtimer2_0();
    M1_InitPWM();
    M2_InitPWM();
    M1_InitLPSPI();
    M2_InitLPSPI();
    M1_InitSincFilter();
    M2_InitSincFilter();
#if POSITION_SENSOR == OPTICAL_ENC
    M1_InitQdc();
    M2_InitQdc();
#elif POSITION_SENSOR == TAMAGAWA_ABS_ENC
    M1_InitTamagawaAbs();
    M2_InitTamagawaAbs();
#endif
    qtimer1_1_init();
    qtimer1_2_init();
    qtimer1_3_init();
}
