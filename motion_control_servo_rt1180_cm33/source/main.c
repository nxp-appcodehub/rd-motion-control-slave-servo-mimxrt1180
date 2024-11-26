/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "multicore.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "ecat_def.h"
#include "ecatappl.h"
#include "ecat_hw.h"
#include "api_motorcontrol.h"
#include "freemaster.h"
#include "fsl_lpuart.h"
#include "fsl_lpit.h"
#include "fsl_xbar.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RAM_FUNC_CRITICAL __attribute__((section(".ramfunc.$SRAM_ITC_cm33")))
#define LPIT_SOURCECLOCK CLOCK_GetRootClockFreq(kCLOCK_Root_Bus_Aon)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BOARD_InitUART(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
uint16_t ui16PitCnt;
bool bM1CmdUpdateFlag,bM2CmdUpdateFlag;
extern mc_motor_command_t sMotorCmd[2];

void EtherCAT_Init(void)
{
	ECAT_main();
}

/*!
 * @brief Main function
 */
volatile uint8_t test = 0;

int main(void)
{
	(void)MCMGR_EarlyInit();
    /* Init board hardware.*/
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
//    BOARD_InitDebugConsole();

    /* Init core0 and start core1, install core communication table */
    installCoreCommTable();

    /* Workaround: Disable interrupt which might be enabled by ROM. */
    RGPIO_SetPinInterruptConfig(RGPIO1, 4U, kRGPIO_InterruptOutput0, kRGPIO_InterruptOrDMADisabled);
    NVIC_ClearPendingIRQ(GPIO1_0_IRQn);

    /* Wait until the secondary core application signals that it has been started. */
    while (ui16SlaveReady != 1)
    {
    };
    (void)PRINTF("[Master]: The secondary core application has been started.\r\n");


    EnableIRQ(BOARD_USER_BUTTON_IRQ);

    BOARD_InitUART();
    FMSTR_Init();


#if 1
#if 0
    /* Route LPIT0_CHANNAL0 output to XBAR_OUT0 and XBAR_OUT1 */
    //XBAR_SetSignalsConnection(kXBAR1_InputPit1Trigger0, kXBAR1_OutputDma4MuxReq154);
    //XBAR_SetSignalsConnection(kXBAR1_InputPit1Trigger0, kXBAR1_OutputDma4MuxReq155);

    /* Structure of initialize LPIT */
    lpit_config_t lpitConfig;
    lpit_chnl_params_t lpitChannelConfig;
    /*
     * lpitConfig.enableRunInDebug = false;
     * lpitConfig.enableRunInDoze = false;
     */
    LPIT_GetDefaultConfig(&lpitConfig);

    /* Init lpit module */
    LPIT_Init(LPIT1, &lpitConfig);
    lpitChannelConfig.chainChannel          = false;
    lpitChannelConfig.enableReloadOnTrigger = false;
    lpitChannelConfig.enableStartOnTrigger  = false;
    lpitChannelConfig.enableStopOnTimeout   = false;
    lpitChannelConfig.timerMode             = kLPIT_PeriodicCounter;
    /* Set default values for the trigger source */
    lpitChannelConfig.triggerSelect = kLPIT_Trigger_TimerChn0;
    lpitChannelConfig.triggerSource = kLPIT_TriggerSource_External;

    /* Init lpit channel 0 */
    LPIT_SetupChannel(LPIT1, kLPIT_Chnl_0, &lpitChannelConfig);

    /* Set timer period for channel 0 */
    LPIT_SetTimerPeriod(LPIT1, kLPIT_Chnl_0, USEC_TO_COUNT(125U, LPIT_SOURCECLOCK));

    /* Enable timer interrupts for channel 0 */
    //LPIT_EnableInterrupts(LPIT1, kLPIT_Channel0TimerInterruptEnable);

    /* Enable at the NVIC */
    //EnableIRQWithPriority(LPIT1_IRQn, 0);

    //LPIT_StartTimer(LPIT1, kLPIT_Chnl_0);
#endif
    EtherCAT_Init();
#else
    /* Route LPIT0_CHANNAL0 output to XBAR_OUT0 and XBAR_OUT1 */
       XBAR_SetSignalsConnection(kXBAR1_InputPit1Trigger0, kXBAR1_OutputDma4MuxReq154);
       XBAR_SetSignalsConnection(kXBAR1_InputPit1Trigger0, kXBAR1_OutputDma4MuxReq155);

       /* Structure of initialize LPIT */
       lpit_config_t lpitConfig;
       lpit_chnl_params_t lpitChannelConfig;
       /*
        * lpitConfig.enableRunInDebug = false;
        * lpitConfig.enableRunInDoze = false;
        */
       LPIT_GetDefaultConfig(&lpitConfig);

       /* Init lpit module */
       LPIT_Init(LPIT1, &lpitConfig);
       lpitChannelConfig.chainChannel          = false;
       lpitChannelConfig.enableReloadOnTrigger = false;
       lpitChannelConfig.enableStartOnTrigger  = false;
       lpitChannelConfig.enableStopOnTimeout   = false;
       lpitChannelConfig.timerMode             = kLPIT_PeriodicCounter;
       /* Set default values for the trigger source */
       lpitChannelConfig.triggerSelect = kLPIT_Trigger_TimerChn0;
       lpitChannelConfig.triggerSource = kLPIT_TriggerSource_External;

       /* Init lpit channel 0 */
       LPIT_SetupChannel(LPIT1, kLPIT_Chnl_0, &lpitChannelConfig);

       /* Set timer period for channel 0 */
       LPIT_SetTimerPeriod(LPIT1, kLPIT_Chnl_0, USEC_TO_COUNT(1000U, LPIT_SOURCECLOCK));

       /* Enable timer interrupts for channel 0 */
       LPIT_EnableInterrupts(LPIT1, kLPIT_Channel0TimerInterruptEnable);

       /* Enable at the NVIC */
       EnableIRQWithPriority(LPIT1_IRQn, 0);

       LPIT_StartTimer(LPIT1, kLPIT_Chnl_0);

    while(1)
    {
    	if(bM1CmdUpdateFlag == true)
    	{
//    		MC_SetMotor1Command(&sM1Cmd);
    	    MC_SetMotor1Command(&sMotorCmd[0]);

    		bM1CmdUpdateFlag = false;
    	}
    	if(bM2CmdUpdateFlag == true)
    	{
//    		MC_SetMotor2Command(&sM2Cmd);
    	    MC_SetMotor2Command(&sMotorCmd[1]);
    		bM2CmdUpdateFlag = false;
    	}
    	FMSTR_Poll();
    }
#endif
}

/*!
 *@brief      Initialization of the UART module
 *
 *@param      u32BaudRate         Baud rate
 *
 *@return     none
 */
static void BOARD_InitUART(void)
{
    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(BOARD_DEBUG_UART_BASEADDR, &config, BOARD_DebugConsoleSrcFreq());

    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress(BOARD_DEBUG_UART_BASEADDR);

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    /* Enable UART interrupts. */
    EnableIRQ(BOARD_FMSTR_UART_IRQ);
    NVIC_SetPriority(BOARD_FMSTR_UART_IRQ, 6);
#endif
}

mc_motor_command_t g_sM1Cmd, g_sM2Cmd;
void LPIT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    LPIT_ClearStatusFlags(LPIT1, kLPIT_Channel0TimerFlag);
//    MC_SetMotorCommandFromISR();

    if(++ui16PitCnt == 8001)
    {
    	ui16PitCnt = 0;
    }

    SDK_ISR_EXIT_BARRIER;
}

void BOARD_USER_BUTTON_IRQ_HANDLER(void)
{
    /* Clear external interrupt flag. */
    RGPIO_ClearPinsInterruptFlags(BOARD_USER_BUTTON_GPIO, kRGPIO_InterruptOutput0, 1U << BOARD_USER_BUTTON_GPIO_PIN);
    /* Change state of button. */

    SDK_ISR_EXIT_BARRIER;
}
