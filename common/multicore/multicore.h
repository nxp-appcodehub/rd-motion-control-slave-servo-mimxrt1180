/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MULTICORE_H_
#define MULTICORE_H_

#include <stdbool.h>
#include <stdint.h>
#include "fsl_common.h"
#include "mcmgr.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @brief Trigger event
 *
 * This function is used to send event to remote core.
 *
 * @param[in] eventData - Data to send to remote core, defined in app.h.
 *
 * @return kStatus_MCMGR_Success on success or kStatus_MCMGR_Error on failure.
 */
#define SendEventToRemoteCore(event) MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (uint16_t)event)

/* Address of memory, from which the secondary core will boot */
#define CORE1_BOOT_ADDRESS (void *)0x303C0000
#define CORE1_KICKOFF_ADDRESS 0x0

/*!
 * @brief Type definition of event callback function pointer
 *
 * This is event processing function corresponding to the event type, pShareMemoryAddress is passed when calling MCMGR_Core0Init_StartCore1 or MCMGR_Core1Init.
 *
 * @param[in] pShareMemoryAddress - Pointer to the shared memory block base address.
 */
typedef void (*event_process_fuc_t)(void *pShareMemoryAddress);

/* Event type and related process function and next event type*/
typedef struct _event_process
{
	event_type_t         eEventType;       /* event received from remote core */
	event_process_fuc_t  pProcessFunction; /* event process function to do processing when related event received */
	event_type_t         eNextEventType;   /* event sent to remote core after the receiving event is processed */
} event_process_t;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add items to EventTable array
 *
 * This function is used to add the events and the corresponding processing function customized by the user.
 *
 * @param[in] eventType - Enum of the event received from remote core.
 * @param[in] processFunction - Event process function to do processing when related event received.
 * @param[in] nextEventType - Enum of the event sent to remote core after the receiving event is processed
 *
 * @return kStatus_Success on success or kStatus_Fail on failure.
 */
status_t EventTableAddItem(event_type_t eventType, event_process_fuc_t processFunction, event_type_t nextEventType);
/*!
 * @brief Initialize Core0 and start Core1
 *
 * This function is used to initialize Core0 and then start Core1.
 *
 * @param[in] pShareMemoryAddress - Base address of shared memory block between Core0 and Core1，
 * is used in event callback function.
 */
void MCMGR_Core0Init_StartCore1(void *pShareMemoryAddress);
/*!
 * @brief Initialize Core1
 *
 * This function is used to initialize Core1.
 *
 * @param[in] pShareMemoryAddress - Base address of shared memory block between Core0 and Core1，
 * is used in event callback function.
 */
void MCMGR_Core1Init(void *pShareMemoryAddress);

#endif /* MULTICORE_H_ */
