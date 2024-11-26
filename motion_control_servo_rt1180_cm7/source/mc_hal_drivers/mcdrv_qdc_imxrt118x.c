/*
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_qdc_imxrt118x.h"
#include "amclib.h"
#include "mlib.h"
#include "amclib_FP.h"
#include "mlib_FP.h"
#include "mc_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief When initial rotor position has been identified:

         1. Get position counter value, and store it to ui32InitCount
         2. Transform this counter value to mechanical position value, store it to f32PosMechInit, which is Q1.31 format, corresponding to -pi~pi
         3. Store rotor flux initial mechanical position (between rotor flux and stator A-axis) to f32PosMechOffset, which is Q1.31 format.

 * @param this               Pointer to the current object
         f32PosMechOffset   The mechanical angle between rotor flux and stator A-axis when initial rotor position is identified. Q1.31 format.
 * @return none
 */
void MCDRV_GetRotorInitPos(mcdrv_qdc_block_t *this, frac32_t f32PosMechOffset)
{

	/* Get QDC 32bit counter value when rotor initial position is identified, stored in Init Count */
	this->ui16Dummy = this->pQDC_base->LPOS;
	this->ui32InitCount = ((uint32_t)(this->pQDC_base->UPOSH)<<16)|(this->pQDC_base->LPOSH);

	this->f32PosMechInit = ((uint64_t)this->i32Q10Cnt2PosGain * this->ui32InitCount)>>10; /* Q22.10 * Q32 = Q54.10, get rid of the last 10 fractional bits, keeping the last 32bits of Q54, and treating it as Q1.31 */
	this->f32PosMechOffset = f32PosMechOffset;

	this->bPosAbsoluteFlag = TRUE;
}



/*!
 * @brief Get current rotor mechanical/electrical position after rotor initial position has been identified.
         Remember to call MCDRV_GetRotorInitPos before invoking this function.

             1. Get current position counter value, and store it to ui32CurrentCount
             2. Transform this counter value to rotor flux real mechanical/position value (relative to A-axis)

 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_GetRotorCurrentPos(mcdrv_qdc_block_t *this)
{
	frac32_t f32Pos;

	/* Get QDC 32bit counter */
	this->ui16Dummy = this->pQDC_base->LPOS;
	this->ui32CurrentCount = ((uint32_t)(this->pQDC_base->UPOSH)<<16)|(this->pQDC_base->LPOSH);

	f32Pos = ((uint64_t)this->i32Q10Cnt2PosGain * this->ui32CurrentCount)>>10; /* Q22.10 * Q32 = Q54.10, get rid of the last 10 fractional bits, keeping the last 32bits of Q54
	                                                                        think of this result as a Q1.31 format, which represents -pi ~ pi */
	this->f32PosMech = f32Pos - this->f32PosMechInit + this->f32PosMechOffset;
	this->f32PosElec = (uint64_t)this->f32PosMech * this->ui16PolePair;
	this->f16PosElec = MLIB_Conv_F16l(this->f32PosElec);

}

/*!
 * @brief Get the initial revolution value (Q16.16 format) from REV counter and position counter, store it to i32Q16InitRev. This initial revolution value will be used as a base.
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_GetRotorInitRev(mcdrv_qdc_block_t *this)
{
	uint32_t ui32Tmp;

	/* Get current QDC 32bit counter value */
	this->ui16Dummy = this->pQDC_base->LPOS;
	ui32Tmp = ((uint32_t)(this->pQDC_base->UPOSH)<<16)|(this->pQDC_base->LPOSH);

	/* Get the revolution value from QDC when rotor initial position is identified. */
	ui32Tmp = ui32Tmp * this->f32ReciprocalLines; /* Get fractional part of the revolution, which is a Q1.31 format, and it's always in range [0~1) */
	this->i32Q16InitRev = ((uint32_t)(this->pQDC_base->REVH)<<16)|(MLIB_Conv_F16l(ui32Tmp)<<1); /* Q16.16 */

}
volatile uint32_t ui32QDCRegTime;
/*!
 * @brief Get current rotor revolution value from Rev counter and position counter, store it to i32Q16Rev, which is Q16.16 format.
         It can represent -32768 ~ 32767.99974 revolutions.

 * @param this   Pointer to the current object
 *
 * @return  none
 */
void MCDRV_GetRotorCurrentRev(mcdrv_qdc_block_t *this)
{
	uint32_t ui32Tmp;

	/* Get current QDC 32bit counter value */
	this->ui16Dummy = this->pQDC_base->LPOS;
	ui32Tmp = ((uint32_t)(this->pQDC_base->UPOSH)<<16)|(this->pQDC_base->LPOSH);

	/* Get the revolution value from QDC when rotor initial position is identified. */
	ui32Tmp = ui32Tmp * this->f32ReciprocalLines; /* Get fractional part of the revolution, which is a Q1.31 format, and it's always in range [0~1) */
	this->i32Q16Rev = ((uint32_t)(this->pQDC_base->REVH)<<16)|(MLIB_Conv_F16l(ui32Tmp)<<1); /* Q16.16 */

}

/*!
 * @brief Initialize QDC driver parameters.
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdcInit(mcdrv_qdc_block_t *this)
{
	bool_t bStatusPass = TRUE;

	this->ui32Mod = 4*this->ui16Line-1;
    this->bPosAbsoluteFlag = FALSE;

	return bStatusPass;
}

/*!
 * @brief Initialize rotor speed calculation(Enhanced M/T method) related parameters.
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdcSpeedCalInit(mcdrv_qdc_block_t *this)
{
	bool_t bStatusPass = TRUE;

	this->sSpeed.f32Speed = 0;
	this->sSpeed.i8SpeedSign = 0;
	this->sSpeed.i8SpeedSign_1 = 0;
	this->sSpeed.ui16Period = 0;
	this->sSpeed.ui16Period_1 = 0;
	this->sSpeed.i16PosDiff = 0;
	this->sSpeed.i32Position = 0;
	this->sSpeed.i32Position_1 = 0;


	this->sSpeed.sQDCSpeedFilter.f16FltBfrX[0] = 0;
	this->sSpeed.sQDCSpeedFilter.f32FltBfrY[0] = 0;

	return bStatusPass;
}

/*!
 * @brief Calculate rotor speed by QDC enhanced M/T speed measurement feature.
 *        The speed is stored in sSpeed.f16SpeedFilt(Q1.15,filtered), sSpeed.fltSpeed(float,filtered) and sSpeed.f32Speed(Q1.31, not filtered).
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_QdcSpeedCalUpdate(mcdrv_qdc_block_t *this)
{
	int64_t i64Numerator;

	/* Read POSDH, POSDPERH and LASTEDGEH */
	this->ui16Dummy = this->pQDC_base->POSD;
	this->sSpeed.i16POSDH = this->pQDC_base->POSDH;
	this->sSpeed.ui16POSDPERH = this->pQDC_base->POSDPERH;
	this->sSpeed.ui16LASTEDGEH = this->pQDC_base->LASTEDGEH;

	/* POSDH == 0? */
	if(this->sSpeed.i16POSDH != 0)
	{
		/* Shaft is moving during speed measurement interval */
		this->sSpeed.i16PosDiff = this->sSpeed.i16POSDH;
		this->sSpeed.ui16Period = this->sSpeed.ui16POSDPERH;
		this->sSpeed.ui16Period_1 = this->sSpeed.ui16Period;

		if(this->sSpeed.i16PosDiff > 0)
		{
			this->sSpeed.i8SpeedSign = 1;
		}
		else
		{
			this->sSpeed.i8SpeedSign = -1;
		}

		if(this->sSpeed.i8SpeedSign == this->sSpeed.i8SpeedSign_1)
		{
			/* Calculate speed */
			i64Numerator = ((int64_t)(this->sSpeed.i16PosDiff) * this->sSpeed.f32SpeedCalConst); /* Q16.0 * Q5.27 = Q21.27 */
			this->sSpeed.f32Speed = (i64Numerator / (uint32_t)(this->sSpeed.ui16Period))<<4; /* Q5.27 -> Q1.31 */
		}
		else
		{
			this->sSpeed.f32Speed = 0;
		}
		this->sSpeed.i8SpeedSign_1 = this->sSpeed.i8SpeedSign;
	}
	else
	{
		/* Shaft is NOT moving during speed measurement interval */
		this->sSpeed.ui16Period = this->sSpeed.ui16LASTEDGEH;

		if((uint32_t)(this->sSpeed.ui16Period) > 0xF000UL)
		{
			/* Shaft hasn't been moving for a long time */
			this->sSpeed.f32Speed = 0;
			this->sSpeed.i8SpeedSign_1 = this->sSpeed.i8SpeedSign;
		}
		else
		{
			/* Speed estimation in low speed region */
			if(this->sSpeed.ui16Period > this->sSpeed.ui16Period_1)
			{
				if(this->sSpeed.i8SpeedSign > 0)
				{
					i64Numerator = ((int64_t)(1) * this->sSpeed.f32SpeedCalConst);
					this->sSpeed.f32Speed = (i64Numerator / (uint32_t)(this->sSpeed.ui16Period))<<4;
				}
				else
				{
					i64Numerator = ((int64_t)(-1) * this->sSpeed.f32SpeedCalConst);
					this->sSpeed.f32Speed = (i64Numerator / (uint32_t)(this->sSpeed.ui16Period))<<4;
				}
			}
		}
	}

	this->sSpeed.f16SpeedFilt = GDFLIB_FilterIIR1_F16(MLIB_Conv_F16l(this->sSpeed.f32Speed), &this->sSpeed.sQDCSpeedFilter);
	this->sSpeed.fltSpeed = MLIB_ConvSc_FLTsf(this->sSpeed.f16SpeedFilt, this->sSpeed.fltSpeedFrac16ToAngularCoeff);

}

/*!
 * @brief Initialize rotor speed calculation(Tracking Observer method) related parameters.
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t true on success
 */
bool_t MCDRV_QdcToSpeedCalInit(mcdrv_qdc_block_t *this)
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
void MCDRV_QdcToSpeedCalUpdate(mcdrv_qdc_block_t *this)
{
	this->sSpeedEstim.f16PosErr = MLIB_Sub_F16(this->f16PosElec, this->sSpeedEstim.f16PosEstim);
	this->sSpeedEstim.f16PosEstim = AMCLIB_TrackObsrv_A32af((acc32_t)(this->sSpeedEstim.f16PosErr), &this->sSpeedEstim.sTO);
	this->sSpeedEstim.fltSpeedEstim = this->sSpeedEstim.sTO.fltSpeed;
}





