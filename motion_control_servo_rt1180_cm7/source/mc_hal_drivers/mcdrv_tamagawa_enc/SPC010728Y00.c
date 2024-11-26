/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_rgpio.h"
#include "fsl_edma.h"
#include "crc.h"
#include "mc_periph_init.h"
#include "pin_mux.h"
#include "fsl_memory.h"
#include "SPC010728Y00.h"

RAM_FUNC_CRITICAL static void absSendControlFieldDataOnly(mcdrv_tamagawa_abs_t *this)
{
	while ((this->LPUART->STAT & LPUART_STAT_TDRE_MASK) == 0); // Wait for transmit buffer empty
    this->LPUART->DATA = this->sRequestFrame.ui8ControlField;
    this->LPUART->CTRL |= LPUART_CTRL_TCIE_MASK; // enable transmission complete interrupt
}

RAM_FUNC_CRITICAL static void absSendEepromReadFrame(mcdrv_tamagawa_abs_t *this)
{
	while ((this->LPUART->STAT & LPUART_STAT_TDRE_MASK) == 0); // Wait for transmit buffer empty
    this->LPUART->DATA = this->sRequestFrame.ui8ControlField;
	this->LPUART->DATA = this->sRequestFrame.uEepromAddressField.ui8AddressField;
	this->LPUART->DATA = this->sRequestFrame.ui8CrcField;
    this->LPUART->CTRL |= LPUART_CTRL_TCIE_MASK; // enable transmission complete interrupt
}

RAM_FUNC_CRITICAL static void absSendEepromWriteFrame(mcdrv_tamagawa_abs_t *this)
{
	while ((this->LPUART->STAT & LPUART_STAT_TDRE_MASK) == 0); // Wait for transmit buffer empty
    this->LPUART->DATA = this->sRequestFrame.ui8ControlField;
	this->LPUART->DATA = this->sRequestFrame.uEepromAddressField.ui8AddressField;
	this->LPUART->DATA = this->sRequestFrame.ui8EepromField;
	this->LPUART->DATA = this->sRequestFrame.ui8CrcField;
    this->LPUART->CTRL |= LPUART_CTRL_TCIE_MASK; // enable transmission complete interrupt
}

RAM_FUNC_CRITICAL static void absReceiveFrameLengthConfig4DMA(mcdrv_tamagawa_abs_t *this, uint8_t ui8Val)
{
    DMA4->TCD[this->ui8DMAChannel].CITER_ELINKNO = ui8Val; /* 1 minor loop transfers 1 byte only. Major loop length is the response frame length */
    DMA4->TCD[this->ui8DMAChannel].DLAST_SGA = -ui8Val;
    DMA4->TCD[this->ui8DMAChannel].BITER_ELINKNO = ui8Val; 
    DMA4->TCD[this->ui8DMAChannel].DADDR = this->ui32DADDR; 
}
volatile uint32_t ui32ForceViolationCnt;
RAM_FUNC_CRITICAL void MCDRV_AbsRequestData(mcdrv_tamagawa_abs_t *this)
{
//	if(!(this->pGetPWMForceFlag() && this->ui8RequestAckFlag == SEND_REQEUST)) // Don't send anything when PWM is just forced and a communication is in progress
//	{
		if(this->ui8RequestAckFlag == SEND_REQEUST)
		{
			// Request is triggered again during a transmission, or the cable is disconnected
//			this->ui16Status |= COMM_CABLE_DISCONNECTED;
		}
		else
		{

			this->ui8RequestAckFlag = SEND_REQEUST;
			switch (this->sRequestFrame.ui8ControlField)
			{
			case READ_DATA_ANGLE:
			case READ_DATA_TURNS:
			case RESET_ALL_ERROR:
			case RESET_ONE_REVOLUTION:
			case RESET_ALL_ERROR_TURNS:
				this->pResponseFrameLengthConfig(this, 6); // 6 Bytes to receive
				break;
			case READ_DATA_ID:
				this->pResponseFrameLengthConfig(this, 4); // 4 Bytes to receive
				break;
			case READ_DATA_ALL:
				this->pResponseFrameLengthConfig(this, 11); // 11 Bytes to receive
				break;
			default:
				break;
			}
			this->pTransmitEnable();    // enable output direction
			this->pSendCFRequest(this); // send data ID
		}


//	}
//	else
//	{
//		ui32ForceViolationCnt++;
//	}
//	this->pClearPWMForceFlag();

}

RAM_FUNC_CRITICAL void MCDRV_AbsRequestEepromRead(mcdrv_tamagawa_abs_t *this)
{
	uint8_t ui8Array[2];
//	if(!(this->pGetPWMForceFlag() && this->ui8RequestAckFlag == SEND_REQEUST)) // Don't send anything when PWM is just forced and a communication is in progress
//	{
		if(this->ui8RequestAckFlag == SEND_REQEUST)
		{
			this->ui16Status |= COMM_CABLE_DISCONNECTED;
		}

		this->ui8RequestAckFlag = SEND_REQEUST;
		this->pResponseFrameLengthConfig(this, 4); // 4 Bytes to receive

		ui8Array[0] = this->sRequestFrame.ui8ControlField;
		ui8Array[1] = this->sRequestFrame.uEepromAddressField.ui8AddressField;
		this->sRequestFrame.ui8CrcField = crc8((uint8_t *)ui8Array, 2);

		this->pTransmitEnable();								   // enable output direction
		this->pSendEepromReadRequest(this);
//	}
//	this->pClearPWMForceFlag();
}

RAM_FUNC_CRITICAL void MCDRV_AbsRequestEepromWrite(mcdrv_tamagawa_abs_t *this)
{
	uint8_t ui8Array[3];
//	if(!(this->pGetPWMForceFlag() && this->ui8RequestAckFlag == SEND_REQEUST)) // Don't send anything when PWM is just forced and a communication is in progress
//	{
		if(this->ui8RequestAckFlag == SEND_REQEUST)
		{
			this->ui16Status |= COMM_CABLE_DISCONNECTED;
		}

		this->ui8RequestAckFlag = SEND_REQEUST;
		this->pResponseFrameLengthConfig(this, 4); // 4 Bytes to receive

		ui8Array[0] = this->sRequestFrame.ui8ControlField;
		ui8Array[1] = this->sRequestFrame.uEepromAddressField.ui8AddressField;
		ui8Array[2] = this->sRequestFrame.ui8EepromField;
		this->sRequestFrame.ui8CrcField = crc8((uint8_t *)ui8Array, 3);

		this->pTransmitEnable();								   // enable output direction
		this->pSendEepromWriteRequest(this);
//	}
//	this->pClearPWMForceFlag();
}

RAM_FUNC_CRITICAL void MCDRV_AbsGetDataResponse(mcdrv_tamagawa_abs_t *this)
{
	uint8_t u8Crc;
	this->ui16Status = 0;

	switch (this->sRequestFrame.ui8ControlField)
	{
		case READ_DATA_ANGLE:
		case READ_DATA_TURNS:
		case RESET_ALL_ERROR:
		case RESET_ONE_REVOLUTION:
		case RESET_ALL_ERROR_TURNS:
			u8Crc = crc8((uint8_t *)(&this->uResponseFrame.ui8DataReceiveArray[0]), 6);
			break;
		case READ_DATA_ID:
			u8Crc = crc8((uint8_t *)(&this->uResponseFrame.ui8DataReceiveArray[0]), 4);
			break;
		case READ_DATA_ALL:
			u8Crc = crc8((uint8_t *)(&this->uResponseFrame.ui8DataReceiveArray[0]), 11);
			this->ui16Status = this->uResponseFrame.sReceivedData.ui8DataField7; // Get error from ALMC field
			break;
		default:
			break;
	}

	if (u8Crc != 0)
	{
		this->ui16Status |= CRC_ERROR;
	}
	if (this->uResponseFrame.sReceivedData.uStatusField.Bits.CommParityError)
	{
		this->ui16Status |= COMM_PARITY_ERROR;
	}
	if (this->uResponseFrame.sReceivedData.uStatusField.Bits.CommDelimiterError)
	{
		this->ui16Status |= COMM_DELIMITER_ERROR;
	}
	if (this->uResponseFrame.sReceivedData.uStatusField.Bits.CountingError)
	{
		this->ui16Status |= COUNTING_ERROR;
	}
	this->ui8RequestAckFlag = REQUEST_ACKNOWLEDGED;
}

RAM_FUNC_CRITICAL uint8_t MCDRV_AbsGetEepromResponse(mcdrv_tamagawa_abs_t *this)
{
	uint8_t ui8Crc;
	this->ui16Status = 0;

	ui8Crc = crc8((uint8_t *)(&this->uResponseFrame.ui8DataReceiveArray[0]), 4);

	if (ui8Crc != 0)
	{
		this->ui16Status |= CRC_ERROR;
	}
	if (this->uResponseFrame.sReceivedData.uStatusField.uEepromAddr.Bits.BusyFlag)
	{
		this->ui16Status |= EEPROM_BUSY;
	}
	
	ui8Crc = this->uResponseFrame.sReceivedData.ui8DataField0; // Get eeprom data
	this->ui8RequestAckFlag = REQUEST_ACKNOWLEDGED;
	return ui8Crc;
}

/*!
 * @brief Calculate rotor speed by M speed measurement feature for ABS encoder.
 *        The speed is stored in sSpeed.fltSpeed(float, not filtered) and sSpeed.f32Speed(Q1.31, not filtered).
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
int32_t f32PosTest_1, f32PosTest,f32PosTestDelta, f32Pos, f32Pos_1;
RAM_FUNC_CRITICAL void MCDRV_AbsSpeedCalculation(mcdrv_tamagawa_abs_t *this)
{
	int32_t f32PosDelta,i32Temp1,i32Temp2; // Q1.31
	int64_t i64Tmp;

	/* Get position counter raw value and store it to the lower 24bits of a 32bit variable */
	this->sSpeed.ui32RawPosCnt = (((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField2 << 16)
                           | ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField1 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField0);

	i32Temp1 = (((int32_t)(this->sSpeed.ui32RawPosCnt)) << (32 - this->ui8OneTurnBitNum)) ;
	i32Temp2 = ( ((int32_t)(this->sSpeed.ui32RawPosCnt_1)) << (32 - this->ui8OneTurnBitNum)) ;
	f32PosDelta = i32Temp1 - i32Temp2;
	i64Tmp = ((int64_t)(f32PosDelta) * this->sSpeed.i32Q23SpeedCalConst)<<9; // Q1.31*Q9.23=Q10.54 -> Q1.63
	this->sSpeed.f32Speed = (int32_t)(i64Tmp >> 32);
	this->sSpeed.f16Speed    = MLIB_Conv_F16l(this->sSpeed.f32Speed);
//	this->sSpeed.fltSpeed = MLIB_ConvSc_FLTsf(this->sSpeed.f16Speed, this->sSpeed.fltSpeedElecRadScale);   // rad/s elec
	this->sSpeed.fltSpeed = MLIB_ConvSc_FLTlf(this->sSpeed.f32Speed, this->sSpeed.fltSpeedElecRadScale);   // rad/s elec
	this->sSpeed.ui32RawPosCnt_1 = this->sSpeed.ui32RawPosCnt;


}

/*!
 * @brief Initialize rotor speed calculation(Tracking Observer method) related parameters.
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_AbsToSpeedCalInit(mcdrv_tamagawa_abs_t *this)
{
	bool_t bStatusPass = TRUE;

	AMCLIB_TrackObsrvInit_A32af(ACC32(0), &this->sSpeedEstim.sTO);
	this->sSpeedEstim.f16PosErr = 0;
	this->sSpeedEstim.f16PosEstim = 0;
	this->sSpeedEstim.fltSpeedEstim = 0;

	return bStatusPass;
}

/*!
 * @brief Calculate rotor speed by tracking observer method.
 *        The speed is stored in sSpeedEstim.fltSpeedEstim(float,not filtered).
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_AbsToSpeedCalUpdate(mcdrv_tamagawa_abs_t *this)
{
	this->sSpeedEstim.f16PosErr = MLIB_Sub_F16(this->f16PosElec, this->sSpeedEstim.f16PosEstim);
	this->sSpeedEstim.f16PosEstim = AMCLIB_TrackObsrv_A32af((acc32_t)(this->sSpeedEstim.f16PosErr), &this->sSpeedEstim.sTO);
	this->sSpeedEstim.fltSpeedEstim = this->sSpeedEstim.sTO.fltSpeed;
}


/*!
 * @brief When initial rotor position has been identified:

		 1. Get position counter value, and store it to ui32InitCount
		 2. Transform this counter value to mechanical position value, store it to f32PosMechInit, which is Q1.31 format, corresponding to -pi~pi
		 3. Store rotor flux initial mechanical position (between rotor flux and stator A-axis) to f32PosMechOffset, which is Q1.31 format.

 * @param this    Pointer to the Tamagawa object
		 f32PosMechOffset   The mechanical angle between rotor flux and stator A-axis when initial rotor position is identified. Q1.31 format.
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_AbsGetRotorInitPos(mcdrv_tamagawa_abs_t *this, frac32_t f32PosMechOffset)
{

	/* Get position counter raw value and store it to the lower 24bits of a 32bit variable */
	this->ui32InitCount = (((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField2 << 16)
                           | ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField1 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField0);

	this->f32PosMechInit = (this->ui32InitCount) << (32 - this->ui8OneTurnBitNum); /* treating it as Q1.31 */
	this->f32PosMechOffset = f32PosMechOffset;

	this->bPosAbsoluteFlag = TRUE;
}

/*!
 * @brief Get current rotor mechanical/electrical position after rotor initial position has been identified.
		 Remember to call MCDRV_AbsGetRotorInitPos before invoking this function.

			 1. Get current position counter value, and store it to ui32CurrentCount
			 2. Transform this counter value to rotor flux real mechanical/position value (relative to A-axis)

 * @param this   Pointer to the Tamagawa object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_AbsGetRotorCurrentPos(mcdrv_tamagawa_abs_t *this)
{
	frac32_t f32Pos;
	/* Get position counter raw value and store it to the lower 24bits of a 32bit variable */
	this->ui32CurrentCount = (((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField2 << 16)
                           | ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField1 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField0);

	f32Pos = (this->ui32CurrentCount) << (32 - this->ui8OneTurnBitNum); /* Enlarge the counter value to occupy all 32bits. Think of this result as a Q1.31 format, which represents -pi ~ pi */
	this->f32PosMech = f32Pos - this->f32PosMechInit + this->f32PosMechOffset;
	this->f32PosElec = (uint64_t)this->f32PosMech * this->ui8PolePair;
	this->f16PosElec = MLIB_Conv_F16l(this->f32PosElec);
	this->f32Q16DeltaPosMech = this->f32PosMech - this->f32PosMech_1;
	this->f32PosMech_1 = this->f32PosMech;
}

/*!
 * @brief Get the initial revolution value (Q16.16 format) from REV counter and position counter, store it to i32Q16InitRev. This initial revolution value will be used as a base.
 *
 * @param this   Pointer to the Tamagawa object. Used for READ_DATA_ALL request only.
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_AbsGetRotorInitRev(mcdrv_tamagawa_abs_t *this)
{
	uint32_t ui32CurrentCount;

	/* Get position counter raw value and store it to the lower 24bits of a 32bit variable */
	this->ui32PosCurCnt = (((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField2 << 16)
                           | ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField1 << 8)
                           | this->uResponseFrame.sReceivedData.ui8DataField0);

	/* Get revolution counter raw value and store it to a 16bit variable */
	this->ui16RevCnt = ( ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField5 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField4);
	this->f32PosMechOffsetforAlign = this->ui32PosCurCnt;
	this->i32Q16InitRev = ((uint32_t)(this->ui16RevCnt)<<16)|(this->ui32PosCurCnt >> (this->ui8OneTurnBitNum - 16)); /* Q16.16 */
}

/*!
 * @brief Get current rotor revolution value from Rev counter and position counter, store it to i32Q16Rev, which is Q16.16 format.
		 It can represent -32768 ~ 32767.99974 revolutions.

 * @param this   Pointer to the Tamagawa object. Used for READ_DATA_ALL request only.
 *
 * @return  none
 */
RAM_FUNC_CRITICAL void MCDRV_AbsGetRotorCurrentRev(mcdrv_tamagawa_abs_t *this)
{
	uint32_t ui32CurrentCount;

	/* Get position counter raw value and store it to the lower 24bits of a 32bit variable */
	this->ui32PosCurCnt = (((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField2 << 16)
                           | ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField1 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField0);

	/* Get revolution counter raw value and store it to a 16bit variable */
	this->ui16RevCnt = ( ((uint32_t)this->uResponseFrame.sReceivedData.ui8DataField5 << 8) 
                           | this->uResponseFrame.sReceivedData.ui8DataField4);

	this->i32Q16Rev = ((uint32_t)(this->ui16RevCnt)<<16)|(this->ui32PosCurCnt >> (this->ui8OneTurnBitNum - 16)); /* Q16.16 */
}

/*!
 * @brief Initialize Tamagawa driver parameters.
 *
 * @param this   Pointer to the Tamagawa object
 *
 * @return boot_t true on success
 */
void MCDRV_TamagawaAbsInit(mcdrv_tamagawa_abs_t *this)
{
	this->pSendCFRequest = absSendControlFieldDataOnly;
	this->pSendEepromReadRequest = absSendEepromReadFrame;
	this->pSendEepromWriteRequest = absSendEepromWriteFrame;
	this->pResponseFrameLengthConfig = absReceiveFrameLengthConfig4DMA;
	this->ui32DADDR = MEMORY_ConvertMemoryMapAddress((uint32_t)((uint32_t *)this->uResponseFrame.ui8DataReceiveArray), kMEMORY_Local2DMA);
	this->ui32InitCount = 0;
	this->ui16RevCnt = 0;
	this->i32Q16DeltaRev = 0;
	this->ui8RequestAckFlag = INIT_REQUEST;
	this->ui8IllTriggerCnt = 0;
}
