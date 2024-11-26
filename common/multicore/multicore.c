/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "multicore.h"
#include "fsl_common.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Array used to store events and corresponding events processing functions.
 * User can add array value by calling EventTableAddItem function.
 */
event_process_t Event_ProcessTable[EventTableLength] = {0};

status_t EventTableAddItem(event_type_t eventType, event_process_fuc_t processFunction, event_type_t nextEventType)
{
    if ((eventType == Event_NULL) || (eventType >= EventTableLength))
    {
        return kStatus_Fail;
    }
    /* Install event type first */
    Event_ProcessTable[eventType].eEventType = eventType;
    /* Install the process function */
    Event_ProcessTable[eventType].pProcessFunction = processFunction;
    /* Install next event type */
    Event_ProcessTable[eventType].eNextEventType = nextEventType;

    return kStatus_Success;
}

/*!
 * @brief Callback function when receiving data from remote core.
 */
static void RemoteEventHandler(uint16_t eventData, void *context)
{
    event_type_t eventType = (event_type_t)eventData;

    if ((eventType >= Slave_Ready) && (eventType < EventTableLength))
    {
        if (Event_ProcessTable[eventType].pProcessFunction!= ((void *)0))
        {
            Event_ProcessTable[eventType].pProcessFunction(context);
        }
        if (Event_ProcessTable[eventType].eNextEventType != Event_NULL)
        {
            SendEventToRemoteCore(Event_ProcessTable[eventType].eNextEventType);
        }
    }
}

/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
#if 0 /* Comment this function because it's executed in RAM, while RAM has no codes yet */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    (void)MCMGR_EarlyInit();
#if __CORTEX_M == 33
    Prepare_CM7(CORE1_KICKOFF_ADDRESS);
#endif
}
#endif

void MCMGR_Core0Init_StartCore1(void *pShareMemoryAddress)
{
    /* Initialize MCMGR, install generic event handlers */
    (void)MCMGR_Init();

    /* Register the application event before starting the secondary core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RemoteEventHandler, ((void *)pShareMemoryAddress));

    /* Boot Secondary core application */
    (void)PRINTF("[Master]: Starting Secondary core.\r\n");
    (void)MCMGR_StartCore(kMCMGR_Core1, (void *)(char *)CORE1_BOOT_ADDRESS, 2, kMCMGR_Start_Synchronous);
}

void MCMGR_Core1Init(void *pShareMemoryAddress)
{
	uint32_t startupData, i;
	mcmgr_status_t status;
    SystemCoreClock = CLOCK_GetRootClockFreq(kCLOCK_Root_M7);

    /* Initialize MCMGR, install generic event handlers */
    (void)MCMGR_Init();

    /* Register the application event to handle signal from other core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RemoteEventHandler, ((void *)pShareMemoryAddress));

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);

    PRINTF("[Slave]: Core1 is started.\r\n");
    /* Signal the remote core we are ready by triggering the event and passing the Slave_Ready data */
    SendEventToRemoteCore(Slave_Ready);
}


