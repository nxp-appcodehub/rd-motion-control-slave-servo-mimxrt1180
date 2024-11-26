/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "mcdrv_gd3000.h"

/*******************************************************************************
*
* Copyright 2014 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale License
* distributed with this Material.
* See the LICENSE file distributed for more details.
*
*
****************************************************************************//*!
*
* @file         mcdrv_gd3000.c
*
* @brief
*
*******************************************************************************/

/******************************************************************************
| includes
|----------------------------------------------------------------------------*/


/******************************************************************************
| external declarations
|----------------------------------------------------------------------------*/

/******************************************************************************
| defines and macros                                      (scope: module-local)
|----------------------------------------------------------------------------*/


/******************************************************************************
| typedefs and structures                                 (scope: module-local)
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          (scope: module-exported)
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                             (scope: module-local)
|----------------------------------------------------------------------------*/


/******************************************************************************
| function prototypes                                     (scope: module-local)
|----------------------------------------------------------------------------*/




/******************************************************************************
| function implementations                                (scope: module-local)
|----------------------------------------------------------------------------*/


/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_init(mcdrv_GD3000_t *this)
{
   uint32_t i;  
  uint8_t ui8IntMask0, ui8IntMask1;

    this->resetSetVal(0); // Reset GD3000
    this->enableSetVal(0); // EN1, EN2 are low
    
    for(i=0; i<800*300; i++);
    
    this->resetSetVal(1); // Remove RST
    
    for(i=0; i<800*300; i++);
  
	this->ui8ResetRequest = 0;

	// Clear interrupt flags, set deadtime to 0
	GD3000_clearFlags(this);
	GD3000_setZeroDeadtime(this);

	// Enable interrupts
    this->sConfigData.uInterruptEnable.B.overTemp = 1;
    this->sConfigData.uInterruptEnable.B.desaturation = 1;
    this->sConfigData.uInterruptEnable.B.framingErr = 1;
    this->sConfigData.uInterruptEnable.B.lowVls = 1;
    this->sConfigData.uInterruptEnable.B.overCurrent = 1;
    this->sConfigData.uInterruptEnable.B.phaseErr = 1;
    this->sConfigData.uInterruptEnable.B.resetEvent = 0;
    this->sConfigData.uInterruptEnable.B.writeErr = 1;

    ui8IntMask0 = SET_INT_MASK0_CMD|(this->sConfigData.uInterruptEnable.ui8R & 0x0F);
    ui8IntMask1 = SET_INT_MASK1_CMD|((this->sConfigData.uInterruptEnable.ui8R >> 4) & 0x0F);

    this->sendData(ui8IntMask0);
    this->sendData(ui8IntMask1);

    // Setup mode
    this->sConfigData.uMode.B.disableDesat = 0;
    this->sConfigData.uMode.B.enableFullOn = 0;
    this->sConfigData.uMode.B.lock = 0;
    this->sendData(SET_MODE_CMD|(this->sConfigData.uMode.ui8R & 0x0F));

    // Clear interrupt flags
    GD3000_clearFlags(this);
    
    this->enableSetVal(1); // Set EN1 and EN2
    for(i=0; i<800*300; i++);

}





/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_getSR0(mcdrv_GD3000_t *this)
{

    /* Status Register 0 reading = 0x00 */
    this->sendData(READ_STATUS0_CMD);
    this->sStatus.uStatus0.ui8R = this->sendData(READ_STATUS0_CMD);

}

/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_getSR1(mcdrv_GD3000_t *this)
{
    /* Status Register 1 reading = 0x01 */
    this->sendData(READ_STATUS1_CMD);
    this->sStatus.uStatus1.ui8R = this->sendData(READ_STATUS0_CMD);

}

/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_getSR2(mcdrv_GD3000_t *this)
{

    /* Status Register 2 reading = 0x02 */
    this->sendData(READ_STATUS2_CMD);
    this->sStatus.uStatus2.ui8R = this->sendData(READ_STATUS0_CMD);

}

/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_getSR3(mcdrv_GD3000_t *this)
{

    /* Status Register 3 reading = 0x03 */
    this->sendData(READ_STATUS3_CMD);
    this->sStatus.ui8Status3 = this->sendData(READ_STATUS0_CMD);

}

/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         void

@details
******************************************************************************/
void GD3000_clearFlags(mcdrv_GD3000_t *this)
{

    this->sendData(CLR_INT0_CMD|0xF);
    this->sendData(CLR_INT1_CMD|0xF);

}

/***************************************************************************//*!
@brief

@param[in,out]  this    Pointer to the current object.

@return         tBool

@details
******************************************************************************/
void GD3000_setZeroDeadtime(mcdrv_GD3000_t * this)
{

    this->sendData(SET_DEADTIME_CMD & 0xFE);

}


/* End of file */
