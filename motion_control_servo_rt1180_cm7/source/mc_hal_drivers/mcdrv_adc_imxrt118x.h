/*
 * Copyright 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCDRV_ADC_IMXRT118x_H_
#define _MCDRV_ADC_IMXRT118x_H_

#include "gdflib.h"
#include "mlib_types.h"
#include "gmclib.h"
#include "fsl_device_registers.h"
#include "mc_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define QMC3G      0x5A     /* 3ph currents and bus voltage are sampled simultaneously */
#define RT1180_EVK 0x22     /* 3ph currents are sampled simultaneously in the first conversion slot of a sequence, dc bus voltage is sampled in the second conversion slot of the sequence */
#define CONTROL_BOARD QMC3G

#define TRIGGER_0  0
#define TRIGGER_1  1
#define TRIGGER_2  2
#define TRIGGER_3  3
#define TRIGGER_4  4
#define TRIGGER_5  5
#define TRIGGER_6  6
#define TRIGGER_7  7

#define CMD_1      0
#define CMD_2      1
#define CMD_3      2
#define CMD_4      3
#define CMD_5      4
#define CMD_6      5
#define CMD_7      6
#define CMD_8      7
#define CMD_9      8
#define CMD_10     9
#define CMD_11     10
#define CMD_12     11
#define CMD_13     12
#define CMD_14     13
#define CMD_15     14

#define NO_EXIST       0x0000
#define ADC1_PREFIX    0x1000
#define ADC2_PREFIX    0x2000
#define SIDE_A_PREFIX  0x0100
#define SIDE_B_PREFIX  0x0200
#define CHANNEL_MASK   0x00FF
#define ADC_INSTANCE_MASK 0xF000
#define SIDE_MASK      0x0F00


/*
 ADC1:    Trigger ->     CMD1      ->       CMD2     -> End
                     sideA_data[0]      sideA_data[1]
                     sideB_data[0]      sideB_data[1]

 ADC2:    Trigger ->     CMD1      ->       CMD2     -> End
                     sideA_data[0]      sideA_data[1]
                     sideB_data[0]      sideB_data[1]                     
*/
typedef struct single_adc_sequence_info
{
	uint8_t ui8TriggerNum; 			/* Trigger number */
    uint8_t ui8Cmd1stNum;           /* The first command numnber */
    uint8_t ui8Cmd2ndNum;           /* The second command numnber */
    uint16_t ui16CMD1st_sideA_info; /* Channel in the first CMD, side-A */
    uint16_t ui16CMD1st_sideB_info; /* Channel in the first CMD, side-B */
    uint16_t ui16CMD2nd_sideA_info;
    uint16_t ui16CMD2nd_sideB_info;
}single_adc_sequence_info_t;

typedef struct single_adc_
{
    single_adc_sequence_info_t sSequenceInfo;
    uint32_t const volatile*pui32ResultFIFO_sideA; /* Pointer to FIFO0 which contains the conversion results of side A */
    uint32_t const volatile*pui32ResultFIFO_sideB; /* Pointer to FIFO1 which contains the conversion results of side B */
    uint32_t ui32FIFO_sideA_data[2]; /* Read out Raw data from FIFO0 */
    uint32_t ui32FIFO_sideB_data[2]; /* Read out Raw data from FIFO1 */
}single_adc_t;

typedef struct dual_adc_sequence_info
{
    single_adc_t sADC1;
    single_adc_t sADC2;
	uint16_t ui16Average;           /* Average number = 2^ui16Average */

    uint16_t ui16IaChannelInfo;     /* Ia information: which ADC to use, which channel, which side */
    uint16_t ui16IbChannelInfo;     /* Ib information: which ADC to use, which channel, which side */
    uint16_t ui16IcChannelInfo;     /* Ic information: which ADC to use, which channel, which side */
    uint16_t ui16UdcChannelInfo;    /* Udc information: which ADC to use, which channel, which side */
    uint16_t ui16AuxChannelInfo;    /* Auxiliary signal information: which ADC to use, which channel, which side */

}dual_adc_sequence_info_t;


typedef struct pha_ab_calib
{
    GDFLIB_FILTER_MA_T_A32 sFiltPhaA; /* phase A offset filter */
    GDFLIB_FILTER_MA_T_A32 sFiltPhaB; /* phase B offset filter */

    uint16_t ui16CalibPhaA;              /* phase A offset calibration intermediate result */
    uint16_t ui16CalibPhaB;              /* phase B offset calibration intermediate result */

    uint16_t ui16OffsetPhaA;             /* phase A offset result */
    uint16_t ui16OffsetPhaB;             /* phase B offset result */

} pha_ab_calib_t;

typedef struct pha_bc_calib
{
    GDFLIB_FILTER_MA_T_A32 sFiltPhaC; /* phase C offset filter */
    GDFLIB_FILTER_MA_T_A32 sFiltPhaB; /* phase B offset filter */

    uint16_t ui16CalibPhaC;              /* phase C offset calibration intermediate result */
    uint16_t ui16CalibPhaB;              /* phase B offset calibration intermediate result */

    uint16_t ui16OffsetPhaC;             /* phase C offset result */
    uint16_t ui16OffsetPhaB;             /* phase B offset result */

} pha_bc_calib_t;

typedef struct pha_ac_calib
{
    GDFLIB_FILTER_MA_T_A32 sFiltPhaA; /* phase A offset filter */
    GDFLIB_FILTER_MA_T_A32 sFiltPhaC; /* phase C offset filter */

    uint16_t ui16CalibPhaA;              /* phase A offset calibration intermediate result */
    uint16_t ui16CalibPhaC;              /* phase C offset calibration intermediate result */

    uint16_t ui16OffsetPhaA;             /* phase A offset result */
    uint16_t ui16OffsetPhaC;             /* phase C offset result */

} pha_ac_calib_t;

typedef struct mcdrv_adc
{
	GMCLIB_3COOR_T_F16 *psIABC; /* pointer to the global 3-phase currents variables */
	frac16_t *pf16UDcBus;       /* pointer to the global DC bus voltage variable */
	frac16_t *pui16AuxChan;     /* pointer to the global auxiliary signal variable */
    uint16_t *pui16SVMSector;   /* pointer to the global auxiliary signal variable */

    uint16_t ui16OffsetFiltWindow; /* ADC Offset filter window */
    pha_bc_calib_t sCurrSec16;        /* ADC setting for SVM sectors 1&6 */
    pha_ac_calib_t sCurrSec23;        /* ADC setting for SVM sectors 2&3 */
    pha_ab_calib_t sCurrSec45;        /* ADC setting for SVM sectors 4&5 */

    dual_adc_sequence_info_t sDual_adc_sequence;

    uint32_t *pui32IaRaw;   /* Raw Ia data from the FIFO */
    uint32_t *pui32IbRaw;   /* Raw Ib data from the FIFO */
    uint32_t *pui32IcRaw;   /* Raw Ic data from the FIFO */
    uint32_t *pui32UdcRaw;  /* Raw Udc data from the FIFO */
    uint32_t *pui32AuxRaw;  /* Raw Aux data from the FIFO */

    uint8_t  ui8MotorNum; 
    acc32_t a32Gain; /* Gain due to difference ADC reference and on-board HW scale */
} mcdrv_adc_t;

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
bool_t MCDRV_AdcRawDataGet(mcdrv_adc_t *this);

/*!
 * @brief Reads and calculates 3 phase samples based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return boot_t True when SVM sector is correct
 */
bool_t MCDRV_Curr3Ph2ShGet(mcdrv_adc_t *this);

/*!
 * @brief Set initial channel assignment for phase currents & DCB voltage

    QMC3G:
        ADC1: TRIGm ->    CMD1 -> END
                       sideA(x)
                       sideB(y)
        ADC2: TRIGm ->    CMD1 -> END
                       sideA(z)
                       sideB(t)
        x,y,z and t represent Ia, Ib, Ic and Udc in any possible sequence  
        
        Incorrect assignments:
        1). Ia, Ib and Ic are assigned to the same ADC
        2). Ia and Ib are on the same ADC, but Udc is on the same ADC as well    
        3). Ic and Ib are on the same ADC, but Udc is on the same ADC as well  
        4). Ia and Ic are on the same ADC, but Udc is on the same ADC as well   

    RT1180-EVK:
        ADC1: TRIGm ->    CMD1   ->  CMD2  -> END
                        sideA(x)    sideA(n1)
                        sideB(y)    sideA(n2)
        ADC2: TRIGm ->    CMD1   ->  CMD2  -> END
                        sideA(z)    sideA(n3)
                        sideB(t)    sideA(n4)
        3 of x,y,z and t represent Ia, Ib, Ic in any possible sequence. Udc can be in any slot within n1~n4.

        Incorrect assignments:
        1). Ia, Ib and Ic are assigned to the same ADC

            ADC input voltage is scaled to 1/2 internally by HW. ADC reference is 1.8V, to make sure input 3.3 will be
            corresponding to 1.8V, SW needs to add another scale.

            C0 = 3.3*1/2 = 1.65
            C1 = 1.8/C0 = 1.8/1.65 = 1.090909, which is the a32Gain


 * @param this   Pointer to the current object
 *
 * @return  True on successful assignment
 */
bool_t MCDRV_Curr3PhDcBusVoltChanAssignInit(mcdrv_adc_t *this);



/*!
 * @brief Initializes phase current channel offset measurement
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_Curr3Ph2ShCalibInit(mcdrv_adc_t *this);

/*!
 * @brief Function reads current offset samples and filter them based on SVM sector
 *
 * @param this   Pointer to the current object
 *
 * @return  True when SVM sector is correct
 */
bool_t MCDRV_Curr3Ph2ShCalib(mcdrv_adc_t *this);

/*!
 * @brief Function passes measured offset values to main structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_Curr3Ph2ShCalibSet(mcdrv_adc_t *this);

/*!
 * @brief Function passes measured offset values to main structure
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_VoltDcBusGet(mcdrv_adc_t *this);

/*!
 * @brief Function reads and passes auxiliary sample
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_AuxValGet(mcdrv_adc_t *this);

/*!
 * @brief Initializes the ADC driver
 *
 * @param this   Pointer to the current object
 *
 * @return none
 */
void MCDRV_ADCDriverInit(mcdrv_adc_t *this);

#ifdef __cplusplus
}
#endif

#endif /* _MCDRV_ADC_IMXRT118x_H_ */
