/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_H_
#define APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Address of share memory, for two cores to access. */
#define SHARE_MEMORY_START_ADDRESS (void *)0x20500000  //OCRAM2 256K

/* value type which located in shared memory */
typedef struct _value_type
{
	uint32_t position; /*!< position value of motor */
	uint32_t speed;   /*!< speed value of motor */
	uint32_t current; /*!< current value of motor */
} value_type_t;

/* Enum of event type send between Master and Slave
 * Event_NULL and Slave_Ready is fixed, user can customize events from 2.
*/
typedef enum _event_type
{
	Event_NULL = 0, /* fixed event */
	Slave_Ready, /* fixed event */
    Slave_SendMotorStatus, /* user customized event */
    Master_SendMotorCmd, /* user customized event */
	EventTableLength /* fixed end value */
} event_type_t;

#endif /* APP_H_ */
