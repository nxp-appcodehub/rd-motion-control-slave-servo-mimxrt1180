/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_xbar.h"
#include "fsl_iomuxc.h"
#include "fsl_ecat.h"
#include "fsl_gpt.h"

#include "ecat_def.h"
#include "ecatslv.h"
#include "ecat_hw.h"
#include "ecatappl.h"
#include "api_motorcontrol.h"
//#include "servo_motor.h"

UINT32 EcatTimerCnt;
extern bool bM1CmdUpdateFlag,bM2CmdUpdateFlag;
extern mc_motor_command_t sM1Cmd, sM2Cmd;
#define ECAT_SYNC_FREQ 1000
static void Ecat_KickOff(void)
{
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_REF_CLK_DIR0_MASK |
                                         BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_REF_CLK_DIR1_MASK;
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |=
        (BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_SEL0_MASK | BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_RMII_SEL1_MASK);

    BLK_CTRL_WAKEUPMIX->MISC_IO_CTRL &= ~(1 << BLK_CTRL_WAKEUPMIX_MISC_IO_CTRL_ECAT_LINK_ACT0_POL_SHIFT);
    BLK_CTRL_WAKEUPMIX->MISC_IO_CTRL &= ~(1 << BLK_CTRL_WAKEUPMIX_MISC_IO_CTRL_ECAT_LINK_ACT1_POL_SHIFT);

	BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= (1 << BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_EEPROM_SIZE_OPTION_SHIFT);

	SRC_GENERAL_REG->SRMASK &= ~(0x1 << SRC_GENERAL_SRTMR_ECAT_RSTO_TRIG_MODE_SHIFT);

    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= (BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_PHY_OFFSET_VEC(2));
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG &= ~BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_GLB_RST_MASK;
    BLK_CTRL_WAKEUPMIX->ECAT_MISC_CFG |= BLK_CTRL_WAKEUPMIX_ECAT_MISC_CFG_GLB_EN_MASK;
}

UINT16 HW_Init(void)
{
    UINT32 intMask;
    UINT16 led_startus = 0;
    xbar_control_config_t xbaraConfig;
    uint32_t gptFreq;
    gpt_config_t gptConfig;
    rgpio_pin_config_t pinConfig = {
        .pinDirection = kRGPIO_DigitalOutput,
        .outputLogic = 0
    };

    /* Init board hardware. */
    //BOARD_InitBootPins();
    //BOARD_ECAT_MDIO_InitPins();
    //BOARD_Port0_RMII_InitPins();
    //BOARD_Port1_RMII_InitPins();
    //BOARD_ECAT_I2C_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();
    SYSTICK_START_COUNT();

    PRINTF("Start the SSC digital_io example...\r\n");

    /* Reset ecat PHY */
    RGPIO_PinInit(RGPIO4, 25, &pinConfig);
    RGPIO_PinInit(RGPIO4, 13, &pinConfig);
    RGPIO_PinWrite(RGPIO4, 25, 0);
    RGPIO_PinWrite(RGPIO4, 13, 0);
    SDK_DelayAtLeastUs(15000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    Ecat_KickOff();

    RGPIO_PinWrite(RGPIO4, 25, 1);
    RGPIO_PinWrite(RGPIO4, 13, 1);
    SDK_DelayAtLeastUs(45000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    SDK_DelayAtLeastUs(45000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    /*set port0 page register*/
    ECAT_EscMdioWrite(ECAT, 0x00, 31, 0x07);

    /*enable prot0 coustomized LED */
    ECAT_EscMdioRead(ECAT, 0x00, 19, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x00, 19, led_startus | (1 << 3));

    /*Set led1 to LINK100 and set led0 to ACK*/
    ECAT_EscMdioRead(ECAT, 0x00, 17, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x00, 17, led_startus | (1 << 3) | (1 << 5));

    /*set port1 page register*/
    ECAT_EscMdioWrite(ECAT, 0x01, 31, 0x07);

    /*enable prot1 coustomized LED */
    ECAT_EscMdioRead(ECAT, 0x01, 19, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x01, 19, led_startus | (1 << 3));

    /*Set led1 to LINK100 and set led0 to ACK*/
    ECAT_EscMdioRead(ECAT, 0x01, 17, &led_startus);
    ECAT_EscMdioWrite(ECAT, 0x01, 17, led_startus | (1 << 3) | (1 << 5));

    RGPIO_PinInit(RGPIO4, 27, &pinConfig);
    RGPIO_PinInit(RGPIO4, 26, &pinConfig);

    /* Disable PDI interrupt */
    ECAT->PDI_AL_EVENT_MASK_PDI &= ~(0x1 << ECAT_PDI_AL_EVENT_MASK_PDI_BF0_SHIFT);



    /*config Sync0/1 IRQ*/
    XBAR_Init(kXBAR_DSC1);

    XBAR_SetSignalsConnection(kXBAR1_InputEcatSyncOut0, kXBAR1_OutputDma4MuxReq154);
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 = 0x0;
    xbaraConfig.activeEdge                   = kXBAR_EdgeRising;
    xbaraConfig.requestType                  = kXBAR_RequestInterruptEnable;
    XBAR_SetOutputSignalConfig(kXBAR1_OutputDma4MuxReq154, &xbaraConfig);
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 |= 0xff << 16;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL1 |= 0xff << 8;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL2 |= 3;
    BLK_CTRL_WAKEUPMIX->XBAR_TRIG_SYNC_CTRL2 |= 3 << 4;

    XBAR_SetSignalsConnection(kXBAR1_InputEcatSyncOut1, kXBAR1_OutputDma4MuxReq155);
    xbaraConfig.activeEdge  = kXBAR_EdgeRising;
    xbaraConfig.requestType = kXBAR_RequestInterruptEnable;
    XBAR_SetOutputSignalConfig(kXBAR1_OutputDma4MuxReq155, &xbaraConfig);

    //DC test Lucas
#if 0
    IOMUXC_SetPinMux(
    		IOMUXC_GPIO_EMC_B2_04_XBAR1_XBAR_INOUT24,
        0U);

    BLK_CTRL_WAKEUPMIX->XBAR_DIR_CTRL1 = BLK_CTRL_WAKEUPMIX_XBAR_DIR_CTRL1_IOMUXC_XBAR_DIR_SEL_24(1);
    XBAR_SetSignalsConnection(kXBAR1_InputEcatSyncOut0, kXBAR1_OutputIomuxXbarInout24);
#endif
    do
    {
        intMask = 0x93;
        HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
        intMask = 0;
        HW_EscReadDWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    } while (intMask != 0x93);

    intMask = 0x00;

    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

    /*Enable GPT1*/
    GPT_GetDefaultConfig(&gptConfig);
    GPT_Init(GPT1, &gptConfig);
    gptFreq = CLOCK_GetRootClockFreq(kCLOCK_Root_Gpt1);
    GPT_SetClockDivider(GPT1, 100);
    GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, gptFreq / 100000);
    GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);
    EnableIRQ(GPT1_IRQn);

    /*Enable PDI IRQ*/
    EnableIRQ(ECAT_INT_IRQn);
    NVIC_EnableIRQ(XBAR1_CH0_CH1_IRQn);
    EnableIRQWithPriority(XBAR1_CH0_CH1_IRQn, 0);
    GPT_StartTimer(GPT1);
	//servo_motor_init();
    return 0;
}

void ECAT_INT_IRQHandler(void)
{
    PDI_Isr();

    SDK_ISR_EXIT_BARRIER;
}
uint16_t ui16CntSync;
uint32_t ui32SyncCycleTime;
/*config Sync0/1 IRQ*/
void XBAR1_CH0_CH1_IRQHandler(void)
{
	SYSTICK_STOP_COUNT(ui32SyncCycleTime);
    SYSTICK_START_COUNT();

    bool status;
    XBAR_GetOutputStatusFlag(kXBAR1_OutputDma4MuxReq154, &status);
    if (status)
    {
        XBAR_ClearOutputStatusFlag(kXBAR1_OutputDma4MuxReq154);
        Sync0_Isr();
    }

    XBAR_GetOutputStatusFlag(kXBAR1_OutputDma4MuxReq155, &status);
    if (status)
    {
        XBAR_ClearOutputStatusFlag(kXBAR1_OutputDma4MuxReq155);
        Sync1_Isr();
    }
    if (++ui16CntSync >= (uint16_t)(ECAT_SYNC_FREQ))
    {
    	ui16CntSync = 0;
        RGPIO_TogglePinsOutput(BOARD_INITPINS_USER_LED2_GPIO, BOARD_INITPINS_USER_LED2_GPIO_PIN_MASK);

    }
    SDK_ISR_EXIT_BARRIER;
}

void HW_Release(void)
{
}

void GPT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);
#if ECAT_TIMER_INT
    ECAT_CheckTimer();
#endif
    EcatTimerCnt++;

    SDK_ISR_EXIT_BARRIER;
}

UINT16 HW_GetTimer(void)
{
    return EcatTimerCnt;
}

void HW_ClearTimer(void)
{
    EcatTimerCnt = 0;
}

void ENABLE_ESC_INT(void)
{
    NVIC_EnableIRQ(ECAT_INT_IRQn);
    NVIC_EnableIRQ(XBAR1_CH0_CH1_IRQn);
    NVIC_EnableIRQ(GPT1_IRQn);
}

void DISABLE_ESC_INT(void)
{
    NVIC_DisableIRQ(XBAR1_CH0_CH1_IRQn);
    NVIC_DisableIRQ(ECAT_INT_IRQn);
    NVIC_DisableIRQ(GPT1_IRQn);
}

void HW_SetLed(UINT8 RunLed, UINT8 ErrorLed)
{
}

void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
