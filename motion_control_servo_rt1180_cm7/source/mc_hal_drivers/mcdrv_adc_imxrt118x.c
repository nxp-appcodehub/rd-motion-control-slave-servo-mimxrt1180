/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcdrv_adc_imxrt118x.h"
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
 * @brief Reads side_A data out of the FIFO0, and side_B data out of FIFO1
 *
 *                ADC1_FIFO0(side_A):  |1st CMD(data[0])|2nd CMD(data[1])|
 *                ADC1_FIFO1(side_B):  |1st CMD(data[0])|2nd CMD(data[1])|
 *                ADC2_FIFO0(side_A):  |1st CMD(data[0])|2nd CMD(data[1])|
 *                ADC2_FIFO1(side_B):  |1st CMD(data[0])|2nd CMD(data[1])|
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t True when the read out data meet the trigger and command 
 */
bool_t MCDRV_AdcRawDataGet(mcdrv_adc_t *this)
{
	bool_t bStatusPass = FALSE;
	bool_t bStatusADC1 = FALSE;
	bool_t bStatusADC2 = FALSE;

	/* Read FIFO */
    this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0] = *this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideA;
    this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0] = *this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideB;
    this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0] = *this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideA;
    this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0] = *this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideB;
	
    this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[1] = *this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideA;
    this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[1] = *this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideB;
    this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[1] = *this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideA;
    this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[1] = *this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideB;

	if((((this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[1] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[1] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[1] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[1] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum+1 ))
	  {
		bStatusADC1 = TRUE;
	  } 

	if((((this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[1] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[1] & ADC_RESFIFO_TSRC_MASK)>>ADC_RESFIFO_TSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[1] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum+1 ) &&\
	  (((this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[1] & ADC_RESFIFO_CMDSRC_MASK)>>ADC_RESFIFO_CMDSRC_SHIFT) == this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum+1 ))
	  {
		bStatusADC2 = TRUE;
	  }   

	  if((bStatusADC1 == TRUE)&&(bStatusADC2 == TRUE))
	  {
		bStatusPass = TRUE;
	  }

	  return bStatusPass;
}

volatile frac16_t f16IcDebug;
volatile frac16_t f16IbDebug;
volatile frac16_t f16IaDebug;
/*!
 * @brief Reads and calculates 3 phase samples based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t True when SVM sector is correct
 */
RAM_FUNC_CRITICAL bool_t MCDRV_Curr3Ph2ShGet(mcdrv_adc_t *this)
{
	bool_t bStatusPass = FALSE;
	GMCLIB_3COOR_T_F16 sIABCtemp;

    int16_t i16Rslt0;
    int16_t i16Rslt1;

#if 1
    switch (*this->pui16SVMSector)
    {
        case 2:
        case 3:
            /* direct sensing of phase A and C, calculation of B */
            i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IaRaw)));
            i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IcRaw)));
            sIABCtemp.f16A = MLIB_Sub_F16(i16Rslt0, this->sCurrSec23.ui16OffsetPhaA)<<1;
            sIABCtemp.f16C = MLIB_Sub_F16(i16Rslt1, this->sCurrSec23.ui16OffsetPhaC)<<1;
            sIABCtemp.f16B = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16C));
            bStatusPass = TRUE;
            break;
        case 4:
        case 5:
            /* direct sensing of phase A and B, calculation of C */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IaRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IbRaw)));
            sIABCtemp.f16A = MLIB_Sub_F16(i16Rslt0, this->sCurrSec45.ui16OffsetPhaA)<<1;
            sIABCtemp.f16B = MLIB_Sub_F16(i16Rslt1, this->sCurrSec45.ui16OffsetPhaB)<<1;
            sIABCtemp.f16C = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16A, sIABCtemp.f16B));
            bStatusPass = TRUE;
            break;
        case 1:
        case 6:
        default:
            /* direct sensing of phase B and C, calculation of A */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IbRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IcRaw)));
            sIABCtemp.f16B = MLIB_Sub_F16(i16Rslt0, this->sCurrSec16.ui16OffsetPhaB)<<1;
            sIABCtemp.f16C = MLIB_Sub_F16(i16Rslt1, this->sCurrSec16.ui16OffsetPhaC)<<1;
            sIABCtemp.f16A = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16B, sIABCtemp.f16C));
            bStatusPass = TRUE;
            break;    
		
    }
#else

                /* direct sensing of phase A and B, calculation of C */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IcRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IbRaw)));
            sIABCtemp.f16C = MLIB_Sub_F16(i16Rslt0, this->sCurrSec23.ui16OffsetPhaC)<<1;
            sIABCtemp.f16B = MLIB_Sub_F16(i16Rslt1, this->sCurrSec45.ui16OffsetPhaB)<<1;
            sIABCtemp.f16A = MLIB_Neg_F16(MLIB_AddSat_F16(sIABCtemp.f16C, sIABCtemp.f16B));
            bStatusPass = TRUE;
#endif

    f16IaDebug = (frac16_t)(*(this->pui32IaRaw)) - this->sCurrSec23.ui16OffsetPhaA;
    f16IbDebug = (frac16_t)(*(this->pui32IbRaw)) - this->sCurrSec45.ui16OffsetPhaB;
    f16IcDebug = (frac16_t)(*(this->pui32IcRaw)) - this->sCurrSec23.ui16OffsetPhaC;;

    this->psIABC->f16A = sIABCtemp.f16A;
    this->psIABC->f16B = sIABCtemp.f16B;
    this->psIABC->f16C = sIABCtemp.f16C;
    return (bStatusPass);
}


bool_t MCDRV_Curr3PhDcBusVoltChanAssignInit(mcdrv_adc_t *this)
{
    bool_t bStatusPass = FALSE;

	uint16_t ui16IaADCInstance, ui16IbADCInstance, ui16IcADCInstance, ui16UdcADCInstance, ui16AuxADCInstance;
    uint16_t ui16IaSide, ui16IbSide, ui16IcSide, ui16UdcSide, ui16AuxSide;

	ui16IaADCInstance = this->sDual_adc_sequence.ui16IaChannelInfo & ADC_INSTANCE_MASK;
	ui16IbADCInstance = this->sDual_adc_sequence.ui16IbChannelInfo & ADC_INSTANCE_MASK;
	ui16IcADCInstance = this->sDual_adc_sequence.ui16IcChannelInfo & ADC_INSTANCE_MASK;
	ui16UdcADCInstance = this->sDual_adc_sequence.ui16UdcChannelInfo & ADC_INSTANCE_MASK;
	ui16AuxADCInstance = this->sDual_adc_sequence.ui16AuxChannelInfo & ADC_INSTANCE_MASK;

    ui16IaSide = this->sDual_adc_sequence.ui16IaChannelInfo & SIDE_MASK;
    ui16IbSide = this->sDual_adc_sequence.ui16IbChannelInfo & SIDE_MASK;
    ui16IcSide = this->sDual_adc_sequence.ui16IcChannelInfo & SIDE_MASK;
    ui16UdcSide = this->sDual_adc_sequence.ui16UdcChannelInfo & SIDE_MASK;
    ui16AuxSide = this->sDual_adc_sequence.ui16AuxChannelInfo & SIDE_MASK;

    this->a32Gain = ACC32(1.0727);

#if CONTROL_BOARD == QMC3G
    if((ui16IaADCInstance == ui16IbADCInstance) && (ui16IcADCInstance == ui16IbADCInstance))
    {
        // Ia, Ib and Ic are assigned to the same ADC. Can't perform simultaneous sampling, return failure
        return bStatusPass;
    }
    else
    {
        if(ui16IaADCInstance == ui16IbADCInstance)
        {
            // Ia and Ib are sampled by the same ADC
            if(ui16IcADCInstance != ui16UdcADCInstance)
            {
                // Ic and Udc are not assigned to the same ADC, return failure
                return bStatusPass;
            }
            else
            {
                if(ui16IaADCInstance == ADC1_PREFIX)
                {
                    // Ia, Ib -> ADC1, Ic, Udc -> ADC2
                    if((ui16IaSide == ui16IbSide) || (ui16IcSide == ui16UdcSide))
                    {
                        // Can't sample Ia&Ib, or Ic&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IcSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC2
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                    }
                }
                else
                {
                    // Ia, Ib -> ADC2, Ic, Udc -> ADC1
                    if((ui16IaSide == ui16IbSide) || (ui16IcSide == ui16UdcSide))
                    {
                        // Can't sample Ia&Ib, or Ic&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IcSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC1
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                    }
                }
            }
        }
        else if(ui16IaADCInstance == ui16IcADCInstance)
        {
            // Ia and Ic are sampled by the same ADC
            if(ui16IbADCInstance != ui16UdcADCInstance)
            {
                // Ib and Udc are not assigned to the same ADC, return failure
                return bStatusPass;
            }
            else
            {
                if(ui16IaADCInstance == ADC1_PREFIX)
                {
                    // Ia, Ic -> ADC1, Ib, Udc -> ADC2
                    if((ui16IaSide == ui16IcSide) || (ui16IbSide == ui16UdcSide))
                    {
                        // Can't sample Ia&Ib, or Ic&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IbSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC2
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                    }
                }
                else
                {
                    // Ia, Ic -> ADC2, Ib, Udc -> ADC1
                    if((ui16IaSide == ui16IcSide) || (ui16IbSide == ui16UdcSide))
                    {
                        // Can't sample Ia&Ib, or Ic&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IbSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC1
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                    }
                }
            }
        }
        else
        {
            // Ib and Ic are sampled by the same ADC
            if(ui16IaADCInstance != ui16UdcADCInstance)
            {
                // Ia and Udc are not assigned to the same ADC, return failure
                return bStatusPass;
            }
            else
            {
                if(ui16IbADCInstance == ADC1_PREFIX)
                {
                    // Ib, Ic -> ADC1, Ia, Udc -> ADC2
                    if((ui16IbSide == ui16IcSide) || (ui16IaSide == ui16UdcSide))
                    {
                        // Can't sample Ib&Ic, or Ia&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IbSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC2
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                    }
                }
                else
                {
                    // Ib, Ic -> ADC2, Ia, Udc -> ADC1
                    if((ui16IbSide == ui16IcSide) || (ui16IaSide == ui16UdcSide))
                    {
                        // Can't sample Ib&Ic, or Ia&Udc simultaneously, return failure
                        return bStatusPass;
                    }
                    else
                    {
                        if(ui16IbSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                            this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                            this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                            this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                        }
                        if(ui16IaSide == SIDE_A_PREFIX)
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideB,ADC1
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                        else
                        {
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo; // Udc sideA,ADC1
                            this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                            this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                            this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                        }
                    }
                }
            }
        }
    }

    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info & CHANNEL_MASK); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info & CHANNEL_MASK); 
    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum+1); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum+1); 


    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideB_info & CHANNEL_MASK); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideB_info & CHANNEL_MASK); 
    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(0); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(0); 

#elif CONTROL_BOARD == RT1180_EVK
    if((ui16IaADCInstance == ui16IbADCInstance) && (ui16IcADCInstance == ui16IbADCInstance))
    {
        // Ia, Ib and Ic are assigned to the same ADC. Can't perform simultaneous sampling, return failure
        return bStatusPass;
    }
    else
    {
        if(ui16IaADCInstance == ui16IbADCInstance)
        {
            // Ia and Ib are sampled by the same ADC
            if(ui16IaADCInstance == ADC1_PREFIX)
            {
                // Ia, Ib -> ADC1, Ic-> ADC2
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                if(ui16IcSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                           
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                    
            }
            else
            {
                // Ia, Ib -> ADC2, Ic -> ADC1
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                if(ui16IcSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];

                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                    
            }
            
        }
        else if(ui16IaADCInstance == ui16IcADCInstance)
        {
            // Ia and Ic are sampled by the same ADC
            if(ui16IaADCInstance == ADC1_PREFIX)
            {
                // Ia, Ic -> ADC1, Ib -> ADC2
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                if(ui16IbSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                
            }
            else
            {
                // Ia, Ic -> ADC2, Ib -> ADC1
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                if(ui16IbSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                
            }
            
        }
        else
        {
            // Ib and Ic are sampled by the same ADC
            if(ui16IbADCInstance == ADC1_PREFIX)
            {
                // Ib, Ic -> ADC1, Ia -> ADC2        
                if(ui16IbSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC1
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC1
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC1
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC2
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC2
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                
            }
            else
            {
                // Ib, Ic -> ADC2, Ia -> ADC1

                if(ui16IbSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideB,ADC2
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IcChannelInfo; // Ic sideA,ADC2
                    this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IbChannelInfo; // Ib sideB,ADC2
                    this->pui32IcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[0];
                    this->pui32IbRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[0];
                }
                if(ui16IaSide == SIDE_A_PREFIX)
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideA,ADC1
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[0];
                }
                else
                {
                    this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info = this->sDual_adc_sequence.ui16IaChannelInfo; // Ia sideB,ADC1
                    this->pui32IaRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[0];
                }
                
            }
            
        }
    }

    switch(ui16UdcADCInstance)
    {
        case ADC1_PREFIX:
        default:
            if(ui16UdcSide == SIDE_A_PREFIX)
            {
                this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo;
                this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideA_data[1];
            }
            else
            {
                this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo;
                this->pui32UdcRaw = &this->sDual_adc_sequence.sADC1.ui32FIFO_sideB_data[1];
            }
        break;
        case ADC2_PREFIX:
            if(ui16UdcSide == SIDE_A_PREFIX)
            {
                this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideA_info = this->sDual_adc_sequence.ui16UdcChannelInfo;
                this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideA_data[1];
            }
            else
            {
                this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideB_info = this->sDual_adc_sequence.ui16UdcChannelInfo;
                this->pui32UdcRaw = &this->sDual_adc_sequence.sADC2.ui32FIFO_sideB_data[1];
            }
        break;
    }

    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD1st_sideB_info & CHANNEL_MASK); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD1st_sideB_info & CHANNEL_MASK); 
    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum+1); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum+1); 

    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui16CMD2nd_sideB_info & CHANNEL_MASK); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum].CMDL = ADC_CMDL_ALTB_CSCALE(0)|ADC_CMDL_ALTBEN_MASK|ADC_CMDL_CSCALE(0)|ADC_CMDL_MODE(0)|ADC_CMDL_CTYPE(3)|\
                                                ADC_CMDL_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideA_info & CHANNEL_MASK)|ADC_CMDL_ALTB_ADCH(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui16CMD2nd_sideB_info & CHANNEL_MASK); 
    ADC1->CMD[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd2ndNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(0); 
    ADC2->CMD[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd2ndNum].CMDH = ADC_CMDH_AVGS(this->sDual_adc_sequence.ui16Average)|ADC_CMDH_NEXT(0); 

#endif
    ADC1->TCTRL[this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8TriggerNum] = ADC_TCTRL_TCMD(this->sDual_adc_sequence.sADC1.sSequenceInfo.ui8Cmd1stNum+1)|ADC_TCTRL_FIFO_SEL_B(1)|ADC_TCTRL_HTEN(1); // use FIFO0 and FIFO1, HW trigger
	ADC2->TCTRL[this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8TriggerNum] = ADC_TCTRL_TCMD(this->sDual_adc_sequence.sADC2.sSequenceInfo.ui8Cmd1stNum+1)|ADC_TCTRL_FIFO_SEL_B(1)|ADC_TCTRL_HTEN(1); // use FIFO0 and FIFO1, HW trigger

    bStatusPass = TRUE;
    return bStatusPass;
}

/*!
 * @brief Initializes the ADC driver
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_ADCDriverInit(mcdrv_adc_t *this)
{

	this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideA = &ADC1->RESFIFO[0];
	this->sDual_adc_sequence.sADC1.pui32ResultFIFO_sideB = &ADC1->RESFIFO[1];


	this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideA = &ADC2->RESFIFO[0];
	this->sDual_adc_sequence.sADC2.pui32ResultFIFO_sideB = &ADC2->RESFIFO[1];
}

/*!
 * @brief Initializes phase current channel offset measurement
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_Curr3Ph2ShCalibInit(mcdrv_adc_t *this)
{

    /* clear offset values */
    this->sCurrSec16.ui16OffsetPhaB = 0;
    this->sCurrSec16.ui16OffsetPhaC = 0;
    this->sCurrSec23.ui16OffsetPhaA = 0;
    this->sCurrSec23.ui16OffsetPhaC = 0;
    this->sCurrSec45.ui16OffsetPhaA = 0;
    this->sCurrSec45.ui16OffsetPhaB = 0;

    this->sCurrSec16.ui16CalibPhaB = 0;
    this->sCurrSec16.ui16CalibPhaC = 0;
    this->sCurrSec23.ui16CalibPhaA = 0;
    this->sCurrSec23.ui16CalibPhaC = 0;
    this->sCurrSec45.ui16CalibPhaA = 0;
    this->sCurrSec45.ui16CalibPhaB = 0;

    /* initialize offset filters */
    this->sCurrSec16.sFiltPhaB.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec16.sFiltPhaC.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec23.sFiltPhaA.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec23.sFiltPhaC.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec45.sFiltPhaA.u16Sh = this->ui16OffsetFiltWindow;
    this->sCurrSec45.sFiltPhaB.u16Sh = this->ui16OffsetFiltWindow;

    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec16.sFiltPhaB);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec16.sFiltPhaC);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec23.sFiltPhaA);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec23.sFiltPhaC);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec45.sFiltPhaA);
    GDFLIB_FilterMAInit_F16((frac16_t)0, &this->sCurrSec45.sFiltPhaB);

}

/*!
 * @brief Function reads current offset samples and filter them based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return  True when SVM sector is correct
 */
RAM_FUNC_CRITICAL bool_t MCDRV_Curr3Ph2ShCalib(mcdrv_adc_t *this)
{
    bool_t bStatusPass = FALSE;

    int16_t i16Rslt0;
    int16_t i16Rslt1;

    switch (*this->pui16SVMSector)
    {
        case 2:
        case 3:
            /* sensing of offset IA -> ADCA and IC -> ADCC */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IaRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IcRaw)));
            this->sCurrSec23.ui16CalibPhaA = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt0), &this->sCurrSec23.sFiltPhaA);
            this->sCurrSec23.ui16CalibPhaC = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt1), &this->sCurrSec23.sFiltPhaC);
            bStatusPass = TRUE;
            break;
        case 4:
        case 5:
            /* sensing of offset IA -> ADCA and IB -> ADCC */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IaRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IbRaw)));
            this->sCurrSec45.ui16CalibPhaA = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt0), &this->sCurrSec45.sFiltPhaA);
            this->sCurrSec45.ui16CalibPhaB = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt1), &this->sCurrSec45.sFiltPhaB);
            bStatusPass = TRUE;
            break;
        case 1:
        case 6:
            /* sensing of offset IB -> ADCA and IC -> ADCC */
        	i16Rslt0 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IbRaw)));
        	i16Rslt1 = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32IcRaw)));
            this->sCurrSec16.ui16CalibPhaB = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt0), &this->sCurrSec16.sFiltPhaB);
            this->sCurrSec16.ui16CalibPhaC = GDFLIB_FilterMA_F16((frac16_t)(i16Rslt1), &this->sCurrSec16.sFiltPhaC);
            bStatusPass = TRUE;
            break;
        default:
        	break;
    }

    return (bStatusPass);
}

/*!
 * @brief Function passes measured offset values to main structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_Curr3Ph2ShCalibSet(mcdrv_adc_t *this)
{

    /* pass calibration data for sector 1 and 6 */
    this->sCurrSec16.ui16OffsetPhaB = this->sCurrSec16.ui16CalibPhaB;
    this->sCurrSec16.ui16OffsetPhaC = this->sCurrSec16.ui16CalibPhaC;

    /* pass calibration data for sector 2 and 3 */
    this->sCurrSec23.ui16OffsetPhaA = this->sCurrSec23.ui16CalibPhaA;
    this->sCurrSec23.ui16OffsetPhaC = this->sCurrSec23.ui16CalibPhaC;

    /* pass calibration data for sector 4 and 5 */
    this->sCurrSec45.ui16OffsetPhaA = this->sCurrSec45.ui16CalibPhaA;
    this->sCurrSec45.ui16OffsetPhaB = this->sCurrSec45.ui16CalibPhaB;

}

/*!
 * @brief Function reads and passes DCB voltage sample
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_VoltDcBusGet(mcdrv_adc_t *this)
{
    int16_t i16Rslt;

    /* read DC-bus voltage sample from defined ADCx result register */
    i16Rslt = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32UdcRaw)));
    *this->pf16UDcBus = (frac16_t)(i16Rslt); /* ADC_ETC trigger0 (ADC1) chain1 */

}

/*!
 * @brief Function reads and passes auxiliary sample
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
RAM_FUNC_CRITICAL void MCDRV_AuxValGet(mcdrv_adc_t *this)
{
    int16_t i16Rslt;

    /* read Auxiliary channel sample from defined ADCx result register */
    i16Rslt = MLIB_Mul_F16as(this->a32Gain, (frac16_t)(*(this->pui32AuxRaw)));
    *this->pui16AuxChan = (frac16_t)(i16Rslt); /* ADC_ETC trigger4 (ADC2) chain1 */
    
}
