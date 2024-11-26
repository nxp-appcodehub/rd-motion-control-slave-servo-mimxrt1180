/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "multicore.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "mcmgr.h"
#include "fsl_debug_console.h"
#include "app.h"

#include "fsl_common.h"
#include "mc_periph_init.h"
#include "freemaster.h"
#include "pin_mux.h"
#include "fsl_rgpio.h"
#include "fsl_lpuart.h"
#include "fsl_edma.h"
#include "fsl_memory.h"
#include "fsl_trdc.h"
#include "state_machine.h"
#include "freemaster_serial_lpuart.h"
#include "board.h"
#include "fsl_cache.h"
#include "api_motorcontrol.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The UART to use for FreeMASTER communication */
#define BOARD_FMSTR_UART_PORT        LPUART1
#define BOARD_FMSTR_UART_BAUDRATE    115200U
#define BOARD_FMSTR_UART_IRQ         LPUART1_IRQn
#define BOARD_FMSTR_UART_IRQ_HANDLER LPUART1_IRQHandler

#define M7_STANDALONE_DEBUG          0
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t ui32ADC1_STAT, ui32ADC2_STAT;
uint8_t ui8M1GD3000RstStatus;
volatile uint32_t ui32M1_FIFOIS, ui32M2_FIFOIS;


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void BOARD_InitUART(uint32_t u32BaudRate);
static void BOARD_InitSysTick(void);
static void TRDC2_EDMA4_ResetPermissions(void);

extern sm_app_ctrl_t g_sM1Ctrl;
extern sm_app_ctrl_t g_sM2Ctrl;


/*******************************************************************************
 * Code
 ******************************************************************************/
static void Fuc2(void *pShareMemoryAddress)
{
	value_type_t *memory = (value_type_t *)pShareMemoryAddress;
	PRINTF("[Slave]: current = %d.\r\n", memory->current);
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t ui32PrimaskReg;
    uint32_t result = 0U;

    /* Init board hardware.*/
    BOARD_ConfigMPU();
#if M7_STANDALONE_DEBUG == 1
    BOARD_InitBootClocks();
#endif
    BOARD_InitBootPins();

    BOARD_InitDebugConsole();

    /* Init UART for FreeMaster communication */
    BOARD_InitUART(BOARD_FMSTR_UART_BAUDRATE);

#if M7_STANDALONE_DEBUG == 0

    /* Init core1, install core communication table */
    installCoreCommTable();

#endif
    /* SysTick initialization for CPU load measurement */
    BOARD_InitSysTick();

    do
    {
        /*Wait TR empty*/
        while ((MU_RT_S3MUA->TSR & MU_TSR_TE0_MASK) == 0)
            ;
        /* Send Get FW Status command(0xc5), message size 0x01 */
        MU_RT_S3MUA->TR[0] = 0x17C50106U;
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF0_MASK) == 0)
            ;
        (void)MU_RT_S3MUA->RR[0];
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF1_MASK) == 0)
            ;
        /* Get response code, only procedd when code is 0xD6 which is S400_SUCCESS_IND. */
        result = MU_RT_S3MUA->RR[1];
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF2_MASK) == 0)
            ;
        (void)MU_RT_S3MUA->RR[2];
    } while (result != 0xD6);

    /*
     * Send Release TRDC command to CM7 Core or CM33
     */
    do
    {
        /*Wait TR empty*/
        while ((MU_RT_S3MUA->TSR & MU_TSR_TE0_MASK) == 0)
            ;
        /* Send release RDC command(0xc4), message size 2 */
        MU_RT_S3MUA->TR[0] = 0x17c40206;
        /*Wait TR empty*/
        while ((MU_RT_S3MUA->TSR & MU_TSR_TE1_MASK) == 0)
            ;
        /* Release TRDC A to the RTD core */
        MU_RT_S3MUA->TR[1] = 0x7402;//the laste 2 bit (cm33, 0x01,CM7 0x02)
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF0_MASK) == 0)
            ;
        (void)MU_RT_S3MUA->RR[0];
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF1_MASK) == 0)
            ;
        result = MU_RT_S3MUA->RR[1];
    } while (result != 0xD6);

    do
    {
        /*Wait TR empty*/
        while ((MU_RT_S3MUA->TSR & MU_TSR_TE0_MASK) == 0)
            ;
        /* Send release RDC command(0xc4), message size 2 */
        MU_RT_S3MUA->TR[0] = 0x17c40206;
        /*Wait TR empty*/
        while ((MU_RT_S3MUA->TSR & MU_TSR_TE1_MASK) == 0)
            ;
        /* Release TRDC A to the RTD core*/
        MU_RT_S3MUA->TR[1] = 0x7802;
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF0_MASK) == 0)
            ;
        (void)MU_RT_S3MUA->RR[0];
        /*Wait RR Full*/
        while ((MU_RT_S3MUA->RSR & MU_RSR_RF1_MASK) == 0)
            ;
        result = MU_RT_S3MUA->RR[1];
    } while (result != 0xD6);

    TRDC2_EDMA4_ResetPermissions();

    /* Disable all interrupts before peripherals are initialized */
    ui32PrimaskReg = DisableGlobalIRQ();

    /* Init peripheral motor control driver for motor M1 */
    peripherals_manual_init();

    /* FreeMaster init */
    FMSTR_Init();

    /* Enable interrupts */
    EnableGlobalIRQ(ui32PrimaskReg);

    /* XBAR_OUT0/1 rising edge triggers flag  */
    XBAR1->CTRL[0] = XBAR_NUM_OUT221_CTRL_EDGE0(1)|XBAR_NUM_OUT221_CTRL_EDGE1(1)|XBAR_NUM_OUT221_CTRL_IEN0_MASK;

    /* Enable fast/slow loop Timer */
    M1_FASTLOOP_TIMER_ENABLE();
    M2_FASTLOOP_TIMER_ENABLE();

    START_TIMER1();
    START_TIMER2();
    START_TIMER3();

    GD3000_init(&g_sM1GD3000);
    GD3000_init(&g_sM2GD3000);

    while (1)
    {
        /* FreeMASTER Polling function */
        FMSTR_Poll();

        ui32ADC1_STAT = ADC1->STAT;
        ui32ADC2_STAT = ADC2->STAT;
        ui32M1_FIFOIS = SINC1->FIFOIS;
        ui32M2_FIFOIS = SINC2->FIFOIS;

        if(g_sM1GD3000.ui8ResetRequest == 1)
        {
            GD3000_init(&g_sM1GD3000);
            g_sM1GD3000.ui8ResetRequest = 0;
        }

        if(g_sM2GD3000.ui8ResetRequest == 1)
        {
            GD3000_init(&g_sM2GD3000);
            g_sM2GD3000.ui8ResetRequest = 0;
        }

        GD3000_getSR0(&g_sM1GD3000);
        GD3000_getSR1(&g_sM1GD3000);
        GD3000_getSR2(&g_sM1GD3000);
        GD3000_getSR3(&g_sM1GD3000);

        GD3000_getSR0(&g_sM2GD3000);
        GD3000_getSR1(&g_sM2GD3000);
        GD3000_getSR2(&g_sM2GD3000);
        GD3000_getSR3(&g_sM2GD3000);

        /*========================= State machine begin========================================================*/
        /* M1 State machine */
        SM_StateMachineFast(&g_sM1Ctrl);
        SM_StateMachineFast(&g_sM2Ctrl);
    }
}

/*!
 *@brief      Initialization of the UART module
 *
 *@param      u32BaudRate         Baud rate
 *
 *@return     none
 */
static void BOARD_InitUART(uint32_t u32BaudRate)
{
    lpuart_config_t config;

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_FMSTR_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    LPUART_Init(BOARD_FMSTR_UART_PORT, &config, BOARD_DebugConsoleSrcFreq());

    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress(BOARD_FMSTR_UART_PORT);

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    /* Enable UART interrupts. */
    EnableIRQ(BOARD_FMSTR_UART_IRQ);
    NVIC_SetPriority(BOARD_FMSTR_UART_IRQ, 6);
#endif
}

/*!
 *@brief      SysTick initialization for CPU cycle measurement
 *
 *@param      none
 *
 *@return     none
 */
static void BOARD_InitSysTick(void)
{
    /* Initialize SysTick core timer to run free */
    /* Set period to maximum value 2^24*/
    SysTick->LOAD = 0xFFFFFF;

    /*Clock source - System Clock*/
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

    /*Start Sys Timer*/
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/*!
 * @brief LPUART handler for FreeMASTER communication
 */
void BOARD_FMSTR_UART_IRQ_HANDLER (void)
{
#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    /* Call FreeMASTER Interrupt routine handler */
    FMSTR_SerialIsr();
#endif
}

static void TRDC2_EDMA4_ResetPermissions(void)
{
    uint8_t i, j;

    /* Set the master domain access configuration for eDMA4 */
    trdc_non_processor_domain_assignment_t edma4Assignment;

    (void)memset(&edma4Assignment, 0, sizeof(edma4Assignment));
    edma4Assignment.domainId       = 0x7U;
    edma4Assignment.privilegeAttr  = kTRDC_MasterPrivilege;
    edma4Assignment.secureAttr     = kTRDC_MasterSecure;
    edma4Assignment.bypassDomainId = true;
    edma4Assignment.lock           = false;
    TRDC_SetNonProcessorDomainAssignment(TRDC2, kTRDC2_MasterDMA4, &edma4Assignment);

    /* Enable all access modes for MBC and MRC of TRDCA and TRDCW */
    trdc_hardware_config_t hwConfig;
    trdc_memory_access_control_config_t memAccessConfig;

    (void)memset(&memAccessConfig, 0, sizeof(memAccessConfig));
    memAccessConfig.nonsecureUsrX  = 1U;
    memAccessConfig.nonsecureUsrW  = 1U;
    memAccessConfig.nonsecureUsrR  = 1U;
    memAccessConfig.nonsecurePrivX = 1U;
    memAccessConfig.nonsecurePrivW = 1U;
    memAccessConfig.nonsecurePrivR = 1U;
    memAccessConfig.secureUsrX     = 1U;
    memAccessConfig.secureUsrW     = 1U;
    memAccessConfig.secureUsrR     = 1U;
    memAccessConfig.securePrivX    = 1U;
    memAccessConfig.securePrivW    = 1U;
    memAccessConfig.securePrivR    = 1U;

    TRDC_GetHardwareConfig(TRDC1, &hwConfig);
    for (i = 0U; i < hwConfig.mrcNumber; i++)
    {
        for (j = 0U; j < 8; j++)
        {
            TRDC_MrcSetMemoryAccessConfig(TRDC1, &memAccessConfig, i, j);
        }
    }

    for (i = 0U; i < hwConfig.mbcNumber; i++)
    {
        for (j = 0U; j < 8; j++)
        {
            TRDC_MbcSetMemoryAccessConfig(TRDC1, &memAccessConfig, i, j);
        }
    }

    TRDC_GetHardwareConfig(TRDC2, &hwConfig);
    for (i = 0U; i < hwConfig.mrcNumber; i++)
    {
        for (j = 0U; j < 8; j++)
        {
            TRDC_MrcSetMemoryAccessConfig(TRDC2, &memAccessConfig, i, j);
        }
    }

    for (i = 0U; i < hwConfig.mbcNumber; i++)
    {
        for (j = 0U; j < 8; j++)
        {
            TRDC_MbcSetMemoryAccessConfig(TRDC2, &memAccessConfig, i, j);
        }
    }
}

