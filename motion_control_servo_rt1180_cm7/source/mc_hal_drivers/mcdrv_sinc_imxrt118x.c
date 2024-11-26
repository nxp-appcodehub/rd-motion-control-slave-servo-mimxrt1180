/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_sinc_imxrt118x.h"
#include "mlib.h"

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
 * @brief   Configure the 4 channel of a Sinc filter with the same order, same decimation rate and the same working mode,
 *          which is continuous mode
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t True when the Fifo depth (decimation_clock_freq/pwm_freq) is an integer,just for the always mode.
 *         when use the continuous/single mode,pass this function.
 */
bool_t MCDRV_SincFilterInit(mcdrv_sinc_t *this)
{
    bool_t bStatusPass = FALSE;

//    if((this->dblFifoDepth - (double)(this->ui8FifoDepth)) != 0)
//    {
//        return bStatusPass;
//    }
//    else
//    {
//        bStatusPass = TRUE;
//    }
    bStatusPass = TRUE;

    this->psSincBaseAddress->MCR = SINC_MCR_MCLK2DIS_MASK|SINC_MCR_MCLK1DIS_MASK|SINC_MCR_MCLKDIV(this->ui8ClkDiv-1)|SINC_MCR_PRESCALE(this->ui8ClkPrescaler); // MCLK0_OUT is 132/2/4 = 16.5MHz
    this->psSincBaseAddress->CHANNEL[this->ui8IaChnNum].CCR = SINC_CCR_PFEN_MASK|SINC_CCR_CHEN_MASK|SINC_CCR_CADEN_MASK|SINC_CCR_SCDEN_MASK;
    this->psSincBaseAddress->CHANNEL[this->ui8IbChnNum].CCR = SINC_CCR_PFEN_MASK|SINC_CCR_CHEN_MASK|SINC_CCR_CADEN_MASK|SINC_CCR_SCDEN_MASK;
    this->psSincBaseAddress->CHANNEL[this->ui8IcChnNum].CCR = SINC_CCR_PFEN_MASK|SINC_CCR_CHEN_MASK|SINC_CCR_CADEN_MASK|SINC_CCR_SCDEN_MASK;
    this->psSincBaseAddress->CHANNEL[this->ui8UdcChnNum].CCR = SINC_CCR_PFEN_MASK|SINC_CCR_CHEN_MASK|SINC_CCR_CADEN_MASK|SINC_CCR_SCDEN_MASK;
    this->psSincBaseAddress->CHANNEL[this->ui8IaChnNum].CDR = SINC_CDR_PFCM(1)|SINC_CDR_PFORD(this->ui8Order)|SINC_CDR_PFOSR(this->ui16DecimationRate-1); // order=3, decimation rate is 16.5MHz/64 = 257.8kHz. FIFO is disabled. Continuous mode.
    this->psSincBaseAddress->CHANNEL[this->ui8IbChnNum].CDR = SINC_CDR_PFCM(1)|SINC_CDR_PFORD(this->ui8Order)|SINC_CDR_PFOSR(this->ui16DecimationRate-1);
    this->psSincBaseAddress->CHANNEL[this->ui8IcChnNum].CDR = SINC_CDR_PFCM(1)|SINC_CDR_PFORD(this->ui8Order)|SINC_CDR_PFOSR(this->ui16DecimationRate-1);
    this->psSincBaseAddress->CHANNEL[this->ui8UdcChnNum].CDR = SINC_CDR_PFCM(1)|SINC_CDR_PFORD(this->ui8Order)|SINC_CDR_PFOSR(this->ui16DecimationRate-1); // Maximum result will be DecimationRate^Order

    this->fltICoeff = this->fltIScale/this->fltRsltScale;
    this->fltUCoeff = this->fltUScale/this->fltRsltScale;
    return bStatusPass;
}




