/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCDRV_SINC_IMXRT118x_H_
#define _MCDRV_SINC_IMXRT118x_H_

#include "gdflib.h"
#include "mlib_types.h"
#include "gmclib.h"
#include "mc_common.h"
#include "fsl_device_registers.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
    SINC_Type *psSincBaseAddress;    /* Sinc filter base address */
    uint8_t ui8IaChnNum;  /* Each sinc module has 4 channels. This is the channel number for Ia */
    uint8_t ui8IbChnNum;  /* Each sinc module has 4 channels. This is the channel number for Ib */
    uint8_t ui8IcChnNum;  /* Each sinc module has 4 channels. This is the channel number for Ic */
    uint8_t ui8UdcChnNum;  /* Each sinc module has 4 channels. This is the channel number for Udc */
    uint8_t ui8Order;     /* Order of this sinc filter */
    uint16_t ui16DecimationRate; /* Decimation rate */ 
    uint8_t ui8ClkPrescaler;
    uint8_t ui8ClkDiv;
    uint8_t ui8FifoDepth; /* Specify how many entries there are in the result FIFO before reading the FIFO */
    int32_t i32IaRawRslt;   /* Desired entry from the FIFO (right shifted) */
    int32_t i32IbRawRslt;   /* Desired entry from the FIFO (right shifted) */
    int32_t i32IcRawRslt;   /* Desired entry from the FIFO (right shifted) */
    int32_t i32UdcRawRslt;   /* Desired entry from the FIFO (right shifted) */
    float   fltIa;
    float   fltIb;
    float   fltIc;
    float   fltUdc;
    float   fltIScale;  /* [A], phase current that corresponds to the maximum measurable range of the external SigmaDelta ADC. */
    float   fltUScale;  /* [V], DC bus voltage that corresponds to the maximum measurable range of the external SigmaDelta ADC. */
    float   fltRsltScale; 
    float   fltICoeff;  /* fltIScale/fltRsltScale */
    float   fltUCoeff;  /* fltUScale/fltRsltScale */
    uint8_t ui8RsltRightShiftBits;


    double dblFifoDepth;  /* This is a double floating type for fifo depth expression, which is used to tell if the configuration is correct: Fifo depth must be an integer based on Sinc clock, PWM 
                           frequency and decimation rate settings */
    double dblSincOutputClockFreq; /* Sinc filter output clock(driving the external SigmaDelta ADC) in Hz, for debug only */
    double dblDecimationClockFreq; /* Decimation clock in Hz, for debug only */
    volatile int32_t i32DummyRslt; /* This field is used to empty the FIFO till the entry that's valid */

} mcdrv_sinc_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @brief Reads and calculates 3 phase currents
 *
 * @param this   Pointer to the current object
 *
 * @return void
 */
ALWAYS_INLINE inline void MCDRV_Get3PhCurrents_with_sw_decimation(mcdrv_sinc_t *this)
{
	uint8_t i;
    for(i=0; i<(this->ui8FifoDepth-1); i++)
    {
        this->i32DummyRslt = this->psSincBaseAddress->CHANNEL[this->ui8IaChnNum].CRDATA;
        this->i32DummyRslt = this->psSincBaseAddress->CHANNEL[this->ui8IbChnNum].CRDATA;
        this->i32DummyRslt = this->psSincBaseAddress->CHANNEL[this->ui8IcChnNum].CRDATA;
    }
    this->i32IaRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IaChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    this->i32IbRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IbChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    this->i32IcRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IcChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    
    this->fltIa = -this->i32IaRawRslt * this->fltICoeff;
    this->fltIb = -this->i32IbRawRslt * this->fltICoeff;
    this->fltIc = -this->i32IcRawRslt * this->fltICoeff;
}

/*!
 * @brief Reads and calculates DC bus voltage
 *
 * @param this   Pointer to the current object
 *
 * @return void
 */
ALWAYS_INLINE inline void MCDRV_GetDcBusVoltage_with_sw_decimation(mcdrv_sinc_t *this)
{
	uint8_t i;
    for(i=0; i<(this->ui8FifoDepth-1); i++)
    {
        this->i32DummyRslt = this->psSincBaseAddress->CHANNEL[this->ui8UdcChnNum].CRDATA;
    }
    this->i32UdcRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8UdcChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    
    this->fltUdc = this->i32UdcRawRslt * this->fltUCoeff;
}

/*!
 * @brief Reads and calculates 3 phase currents
 *
 * @param this   Pointer to the current object
 *
 * @return void
 */
ALWAYS_INLINE inline void MCDRV_Get3PhCurrents_without_sw_decimation(mcdrv_sinc_t *this)
{
    this->i32IaRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IaChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    this->i32IbRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IbChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    this->i32IcRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8IcChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    
    this->fltIa = -this->i32IaRawRslt * this->fltICoeff;
    this->fltIb = -this->i32IbRawRslt * this->fltICoeff;
    this->fltIc = -this->i32IcRawRslt * this->fltICoeff;
}

/*!
 * @brief Reads and calculates DC bus voltage
 *
 * @param this   Pointer to the current object
 *
 * @return void
 */
ALWAYS_INLINE inline void MCDRV_GetDcBusVoltage_without_sw_decimation(mcdrv_sinc_t *this)
{
    this->i32UdcRawRslt = (int32_t)(this->psSincBaseAddress->CHANNEL[this->ui8UdcChnNum].CRDATA)>>(8 + this->ui8RsltRightShiftBits);
    
    this->fltUdc = this->i32UdcRawRslt * this->fltUCoeff;
}

extern bool_t MCDRV_SincFilterInit(mcdrv_sinc_t *this);



#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_SINC_IMXRT118x_H_ */
