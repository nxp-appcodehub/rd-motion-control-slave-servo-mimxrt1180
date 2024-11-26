/*
    * Copyright 2022 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef MOTOR_CONTROL_TASK_MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_MOTOR_CONTROL_TASK_H
#include "mc_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct
{
    /* data */
    float_t fltIa;
    float_t fltIb;
    float_t fltIc;
    float_t fltUdc;
}SINC_RSLT_T;



#define M12_SINC_fastloop_handler    SINC1_CH0_IRQHandler
#define M2_SINC_fastloop_handler    SINC2_CH1_IRQHandler
#define M1_SINC_fastloop_irq		SINC1_CH0_IRQn
#define M2_SINC_fastloop_irq        SINC2_CH1_IRQn

#define M1_ADC_fastloop_handler    ADC1_IRQHandler
#define M2_ADC_fastloop_handler    ADC2_IRQHandler
#define M1_ADC_fastloop_irq		   ADC1_IRQn
#define M2_ADC_fastloop_irq        ADC2_IRQn

#define CMD_Receive_handler    TMR1_IRQHandler

#define M1_slowloop_irq    	   TMR1_IRQn
#define M2_slowloop_irq        TMR2_IRQn

#define M1_M2_slowloop_handler_PWM PWM4_3_IRQHandler
#define M1_M2_fastloop_handler_PWM    PWM2_3_IRQHandler

#define M1_PWM_CMP_handler     PWM4_0_IRQHandler
#define M2_PWM_CMP_handler     PWM2_0_IRQHandler
#define M1_TAMA_ABS_LPUART_TX_handler  LPUART5_IRQHandler
#define M1_TAMA_ABS_DMA_RX_handler     DMA4_CH0_CH1_CH32_CH33_IRQHandler
#define M2_TAMA_ABS_LPUART_TX_handler  LPUART8_IRQHandler
#define M2_TAMA_ABS_DMA_RX_handler     DMA4_CH2_CH3_CH34_CH35_IRQHandler

/*******************************************************************************
 * API
 ******************************************************************************/

extern void M1_fastloop_handler(void);
extern void M2_fastloop_handler(void);
extern void M1_slowloop_handler(void);
extern void M2_slowloop_handler(void);

extern void M1_slowloop_handler_PWM(void);

#ifdef FEATURE_GET_MOTOR_STATUS_FROM_DATA_HUB
extern void getMotorStatusTask(void *pvParameters); // for test purpose
#endif

#endif /* MOTOR_CONTROL_TASK_MOTOR_CONTROL_TASK_H_ */
