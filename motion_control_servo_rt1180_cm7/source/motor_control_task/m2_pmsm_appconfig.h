/*
    * Copyright 2022 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/


#ifndef M2_PMSM_APPCONFIG_H
#define M2_PMSM_APPCONFIG_H

#include "trigonometric.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define MOTOR2_CMD_FROM_FMSTR 0 // 0: from external source; 1: from freemaster

// HW configurations
//-----------------------------------------------------------------------------------------------------------
/*
    Motor2:
  Ia: AD_12 - ADC1_2A
  Ib: AD_13 - ADC1_2B
  Ic: AD_20 - ADC2_1A
  Udc: AD_21 - ADC2_1B
  Idc: AD_23 - ADC2_2B
*/

/* Phase current A channel */
#define M2_I_A (2)
/* Phase current B channel */
#define M2_I_B (2)
/* Phase current C channel */
#define M2_I_C (1)
/*  Udc channel */
#define M2_UDCB (1)
/* Auxiliary signal channel */
#define M2_AUX (2)

#define M2_IA_INFO (M2_I_A|ADC1_PREFIX|SIDE_A_PREFIX)
#define M2_IB_INFO (M2_I_B|ADC1_PREFIX|SIDE_B_PREFIX)
#define M2_IC_INFO (M2_I_C|ADC2_PREFIX|SIDE_A_PREFIX)
#define M2_UDC_INFO (M2_UDCB|ADC2_PREFIX|SIDE_B_PREFIX)
#define M2_AUX_INFO (M2_AUX|ADC2_PREFIX|SIDE_B_PREFIX)

/* Current sampling offset measurement filter window */
#define M2_ADC_OFFSET_WINDOW (3)

#define M2_ADC_AVG_NUM          2      /* 2^AVG_NUM conversions averaged */
#define M2_ADC_TRIGGER_DELAY 	1.0   /* [us] */
#define M2_ADC_AVERAGE_TIME     1.0225 /* [us], 0.2556us per sample */

/* Assignment of eFlexPWM channels to motor 1 phases
 * 0 - PWM channels A0&B0 - sub-module 0
 * 1 - PWM channels A1&B1 - sub-module 1
 * 2 - PWM channels A2&B2 - sub-module 2
 */
#define M2_PWM_PAIR_PHA (0)
#define M2_PWM_PAIR_PHB (1)
#define M2_PWM_PAIR_PHC (2)

/* Over Current Fault detection */
#define M2_FAULT_NUM (0)
#define M2_FAULT_ALT1_NUM  (1)
#define M2_FAULT_ALT2_NUM  (1)


//#define M2_M_2310P_LN_04K
// #define M2_M_2311S_LN_08K
#define M2_M_60ST400
// Motor Parameters 
//-----------------------------------------------------------------------------------------------------------  

#ifdef M2_M_2310P_LN_04K
#define M2_MOTOR_PP 		(4)
#define M2_LD				(0.0002)  /* [H], d-axis inductance */
#define M2_LQ				(0.0002)	  /* [H], q-axis inductance */
#define M2_R				(0.36)	  /* [ohm], phase resistance */
#define M2_ENCODER_LINES	(1000)	  	 /* Encoder lines per mechanical revolution */
#endif

#ifdef M2_M_2311S_LN_08K
#define M2_MOTOR_PP 		(4)
#define M2_LD				(0.001465)  /* [H], d-axis inductance */
#define M2_LQ				(0.001465)	  /* [H], q-axis inductance */
#define M2_R				(1.38)	  /* [ohm], phase resistance */
#define M2_ENCODER_LINES	(2000)	  	 /* Encoder lines per mechanical revolution */
#endif

#ifdef M2_M_60ST400
#define M2_MOTOR_PP 		(5)
#define M2_LD		        (0.00028)  /* [H], d-axis inductance */
#define M2_LQ		        (0.00028)	 /* [H], q-axis inductance */
#define M2_R		        (0.135)	   /* [ohm], phase resistance */
#define M2_ENCODER_LINES	(131072/4)	/* Equivalent Encoder lines per mechanical revolution, not used in TAMAGAWA Abs Hal driver */
#endif

#define M2_N_NOM 			(6000.0F) /* [RPM], motor nominal mechanical speed */
#define M2_I_PH_NOM 		(3.0F)    /* [A], motor nominal current */

// Application Scales 
//-----------------------------------------------------------------------------------------------------------  
#define M2_I_MAX 		(16.665F) /* [A], defined by HW, phase current scale */
#define M2_U_DCB_MAX 	(60.08F)  /* [V], defined by HW, DC bus voltage scale */
#define M2_N_MAX 		(3000.0F) /* [RPM], rotor mechanical speed scale */


#define M2_N_NOM_RAD	(2*PI*M2_N_NOM*M2_MOTOR_PP/60.0)	/* [rad/s], electrical nominal angular speed */
#define M2_U_MAX 		(M2_U_DCB_MAX/1.732F)  				/* [V], phase voltage scale */
#define M2_FREQ_MAX 	(M2_MOTOR_PP*M2_N_MAX/60.0F)  		/* [Hz], electrical speed scale */
#define M2_RAD_MAX		(2*PI*M2_N_MAX*M2_MOTOR_PP/60.0F)	/* [rad/s], electrical angular speed scale */
#define M2_SPEED_FRAC_TO_ANGULAR_COEFF  (float_t)(2*PI*M2_N_MAX*M2_MOTOR_PP/60.0) /* A coefficient that converts scaled speed to electrical angular speed in unit of rad/s */
#define M2_SPEED_MECH_RPM_TO_ELEC_ANGULAR_COEFF (float_t)(2*PI*M2_MOTOR_PP/60.0)
#define M2_SPEED_ELEC_ANGULAR_TO_MECH_RPM_COEFF (float_t)(60.0F / (M2_MOTOR_PP * 2.0F * FLOAT_PI))

// Fault thresholds
//-----------------------------------------------------------------------------------------------------------
#define M2_U_DCB_TRIP 			(28.0F)  	/* [V], brake is on when DC bus reaches this voltage */
#define M2_U_DCB_UNDERVOLTAGE 	(19.0F)  	/* [V], DC bus under voltage threshold */
#define M2_U_DCB_OVERVOLTAGE 	(45.0F)  	/* [V], DC bus over voltage threshold */
#define M2_N_OVERSPEED 			(3500.0F) 	/* [RPM], mechanical over speed threshold */
#define M2_N_MIN 				(300.0F)   	/* [RPM], the minimum mechanical speed that is required for speed command when sensorless approach is used */
  
#define M2_N_OVERSPEED_RAD		(2*PI*M2_N_OVERSPEED*M2_MOTOR_PP/60.0) /* [rad/s], electrical angular over speed threshold */
#define M2_N_MIN_RAD			(2*PI*M2_N_MIN*M2_MOTOR_PP/60.0)	   /* [rad/s], electrical angular minimum speed to ensure sensorless observer can work */

// Control loop frequency
//-----------------------------------------------------------------------------------------------------------
#define M2_PWM_FREQ               (16000.0)   /* [Hz], PWM frequency */
#define M2_FOC_FREQ_VS_PWM_FREQ   (2)         /* FOC calculation is called every n-th PWM reload */
#define M2_PWM_DEADTIME           (0.8)       /* [us], Output PWM deadtime value in micro-seconds */
#define M2_FAST_LOOP_FREQ		  (float_t)(M2_PWM_FREQ*M2_FOC_FREQ_DIV_PWM_FREQ)  /* [Hz], fast loop control frequency */
#define M2_SLOW_LOOP_FREQ		  (8000.0F)   /* [Hz], slow loop control frequency */
#define M2_FOC_FREQ_DIV_PWM_FREQ   (2)         /* FOC calculation is called every n-th PWM reload */


// DC bus voltage filter
//-----------------------------------------------------------------------------------------------------------
#define M2_UDCB_CUTOFF_FREQ  	(160.0F)   /* [Hz], cutoff frequency of IIR1 low pass filter */

#define M2_UDCB_IIR_B0			WARP(M2_UDCB_CUTOFF_FREQ, M2_FAST_LOOP_FREQ)/(WARP(M2_UDCB_CUTOFF_FREQ, M2_FAST_LOOP_FREQ) + 2.0F)
#define M2_UDCB_IIR_B1			WARP(M2_UDCB_CUTOFF_FREQ, M2_FAST_LOOP_FREQ)/(WARP(M2_UDCB_CUTOFF_FREQ, M2_FAST_LOOP_FREQ) + 2.0F)
#define M2_UDCB_IIR_A1			(1.0F - M2_UDCB_IIR_B0 - M2_UDCB_IIR_B1)

  
// Mechanical alignment 
//-----------------------------------------------------------------------------------------------------------  
#define M2_ALIGN_VOLTAGE 		(1.0F)	/* [v], alignment voltage vector length */
#define M2_ALIGN_DURATION_TIME	(1.0F)  /* [s], alignment stage duration */
#define M2_BOOTSTRAP_CHARGE_TIME (0.05) /* [s], charging bootstrap capacitor duration, must be smaller than alignment duration */
#define M2_BOOTSTRAP_DUTY       (0.1)   /* [n/a], duty used in bootstrap capacitor charging */
#define M2_ALIGN_DURATION		(uint16_t)(M2_ALIGN_DURATION_TIME * M2_FAST_LOOP_FREQ) /* Must be less than 32767 */
#define M2_BOOTSTRAP_CHARGE_DURATION   (uint16_t)(M2_BOOTSTRAP_CHARGE_TIME * M2_FAST_LOOP_FREQ)

  
// Application counters 
//-----------------------------------------------------------------------------------------------------------
#define M2_CALIB_DURATION_TIME		(0.5F) /* [s], phase currents calibration stage duration */
#define M2_FAULT_DURATION_TIME  	(6.0F) /* [s], Fault state duration after all faults disappear  */
#define M2_FREEWHEEL_DURATION_TIME 	(1.5F) /* [s], freewheel state duration after motor is stopped from running */

#define M2_CALIB_DURATION			(uint16_t)(M2_CALIB_DURATION_TIME * M2_SLOW_LOOP_FREQ)
#define M2_FAULT_DURATION			(uint16_t)(M2_FAULT_DURATION_TIME * M2_SLOW_LOOP_FREQ)
#define M2_FREEWHEEL_DURATION		(uint16_t)(M2_FREEWHEEL_DURATION_TIME * M2_SLOW_LOOP_FREQ)

  
// Miscellaneous 
//-----------------------------------------------------------------------------------------------------------  
#define M2_E_BLOCK_TRH 			(1.4F)  /* [v], estimated bemf threshold on Q-axis. */
#define M2_E_BLOCK_PER 			(2000)  /* Motor is deemed as blocked when bemf on Q-axis is less than the threshold for this number of times in fast loop  */
#define M2_BLOCK_ROT_FAULT_SH   (1.0F/5) /* filter window */

//Current Loop Control - compared with 2rd order system
//----------------------------------------------------------------------
#define M2_CLOOP_ATT			(0.707F) 	/* attenuation */
#ifdef M2_M_2310P_LN_04K
#define M2_CLOOP_FREQ			(2000.0F)	/* [Hz], oscillating frequency */
#endif

#ifdef M2_M_2311S_LN_08K
#define M2_CLOOP_FREQ			(800.0F)	/* [Hz], oscillating frequency */
#endif

#ifdef M2_M_60ST400
#define M2_CLOOP_FREQ			(2000.0F)	/* [Hz], oscillating frequency */
#endif
//Current Controller Output Limit       
#define M2_CLOOP_LIMIT                     (0.95) /* Voltage output limitation, based on real time available maximum phase voltage amplitude */

#if 0
//D-axis Controller - Parallel type     
#define M2_D_KP_GAIN                       (2*M2_CLOOP_ATT*2*PI*M2_CLOOP_FREQ*M2_LD - M2_R)
#define M2_D_KI_GAIN                       (2*PI*M2_CLOOP_FREQ*2*PI*M2_CLOOP_FREQ*M2_LD/M2_FAST_LOOP_FREQ)
//Q-axis Controller - Parallel type     
#define M2_Q_KP_GAIN                       (2*M2_CLOOP_ATT*2*PI*M2_CLOOP_FREQ*M2_LQ - M2_R)
#define M2_Q_KI_GAIN                       (2*PI*M2_CLOOP_FREQ*2*PI*M2_CLOOP_FREQ*M2_LQ/M2_FAST_LOOP_FREQ)
#else
//D-axis Controller - Parallel type
#define M2_D_KP_GAIN                       (2*PI*M2_CLOOP_FREQ*M2_LD )
#define M2_D_KI_GAIN                       (2*PI*M2_CLOOP_FREQ*M2_R/M2_FAST_LOOP_FREQ)
//Q-axis Controller - Parallel type
#define M2_Q_KP_GAIN                       (2*PI*M2_CLOOP_FREQ*M2_LQ )
#define M2_Q_KI_GAIN                       (2*PI*M2_CLOOP_FREQ*M2_R/M2_FAST_LOOP_FREQ)

#endif

#define M2_CURREQFW_FREQ                   (300.0F)          /* [Hz], cutoff frequency of IIR1 low pass filter for current request from speed pi */

#define M2_CURREQFW_IIR_B0				   WARP(M2_CURREQFW_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_CURREQFW_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_CURREQFW_IIR_B1				   WARP(M2_CURREQFW_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_CURREQFW_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_CURREQFW_IIR_A1				   (1.0F - M1_CURREQFW_IIR_B0 - M1_CURREQFW_IIR_B1)

//Speed Loop Control                    
//----------------------------------------------------------------------
//Speed Controller - Parallel type      
#ifdef M2_M_2310P_LN_04K
  #define M2_SPEED_PI_PROP_GAIN              (0.01F)//(0.25F -> cut feq 250Hz   0.32 -> cut feq 500Hz)		/* proportional gain */0.31    BANDWITH = 270 hz
#endif

#ifdef M2_M_2311S_LN_08K
  #define M2_SPEED_PI_PROP_GAIN              (0.01F)//(0.25F -> cut feq 250Hz   0.32 -> cut feq 500Hz)		/* proportional gain */0.31    BANDWITH = 270 hz
#endif

#ifdef M2_M_60ST400
  #define M2_SPEED_PI_PROP_GAIN              (0.2F)//(0.25F -> cut feq 250Hz   0.32 -> cut feq 500Hz)		/* proportional gain */0.31    BANDWITH = 270 hz
#endif

#define M2_SPEED_PI_INTEG_GAIN             (0.0001F)//(0.00065685F)	/* integral gain */
#define M2_SPEED_PI_PROP_SENSORLESS_GAIN              (0.005F)		/* proportional gain */
#define M2_SPEED_PI_INTEG_SENSORLESS_GAIN             (0.00002F)	/* integral gain */
#define M2_SPEED_PI_DESAT_GAIN			   (0.5F)
#define M2_SPEED_LOOP_HIGH_LIMIT           (7.0F)			/* [A], current output upper limitation */
#define M2_SPEED_LOOP_LOW_LIMIT            (-7.0F)			/* [A], current output lower limitation */
#define M2_CL_SPEED_RAMP			   	   (1000.0F) 		/* [RPM/s], (mechanical) speed accelerating rate during closed-loop */
#define M2_SPEED_CUTOFF_FREQ    		   (300.0F)          /* [Hz], cutoff frequency of IIR1 low pass filter for speed from sensor or observer */
#define M2_SPEED_LOOP_IQ_FWD_GAIN 		   (0.3F)		    /* [A], Iq feed forward gain */
#define M2_SPEED_DEEP_CUTOFF_FREQ    	   (120.0F)          /* [Hz], cutoff frequency of IIR1 low pass filter for speed from sensor or observer */

#define M2_CUR_FWD_CUTOFF_FREQ  		   (300.0F)   /* [Hz], cutoff frequency of IIR1 low pass filter */

#define M2_CUR_FWD_IIR_B0				   WARP(M2_CUR_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_CUR_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_CUR_FWD_IIR_B1				   WARP(M2_CUR_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_CUR_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_CUR_FWD_IIR_A1				   (1.0F - M2_CUR_FWD_IIR_B0 - M2_CUR_FWD_IIR_B1)

#define M2_CL_SPEED_RAMP_RAD			   (2.0F*PI*M2_CL_SPEED_RAMP*M2_MOTOR_PP/60.0) /* transfer mechanical speed in RPM to electrical angular speed */
#define M2_SPEED_RAMP_UP  				   (M2_CL_SPEED_RAMP_RAD/M2_SLOW_LOOP_FREQ)
#define M2_SPEED_RAMP_DOWN  			   (M2_CL_SPEED_RAMP_RAD/M2_SLOW_LOOP_FREQ)

#define M2_SPEED_IIR_B0				   	   WARP(M2_SPEED_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPEED_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPEED_IIR_B1				       WARP(M2_SPEED_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPEED_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPEED_IIR_A1				       (1.0F - M2_SPEED_IIR_B0 - M2_SPEED_IIR_B1)

#define M2_SPEED_DEEP_IIR_B0			   WARP(M2_SPEED_DEEP_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPEED_DEEP_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPEED_DEEP_IIR_B1			   WARP(M2_SPEED_DEEP_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPEED_DEEP_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPEED_DEEP_IIR_A1			   (1.0F - M2_SPEED_DEEP_IIR_B0 - M2_SPEED_DEEP_IIR_B1)

// Position loop control
//-----------------------------------------------------------------------------------------------------------
#define M2_QDC_TRAJECTORY_FILTER_FREQ       4.0 		 /* [Hz], cutoff frequency of a 2nd order IIR filter to get a smoothed position reference */
#define M2_QDC_POSITION_RAMP                30.0 		 /* [Revolutions/s], position ramping rate for a position command */
#define M2_QDC_POSITION_CTRL_P_GAIN			10.0			 /* proportional gain for position controller */
#define M2_QDC_POSITION_CTRL_LIMIT	   		2000.0F		 /* [RPM], mechanical speed - position controller output upper limit. Lower limit is its negative value */
#define M2_QDC_POSITION_CTRL_SPEED_FWD_GAIN 1.0F		 /* Speed feed forward gain */


#define M2_QDC_POSITION_RAMP_UP_FRAC	 	((M2_QDC_POSITION_RAMP/M2_SLOW_LOOP_FREQ)*65535)  /* Q16.16 format */
#define M2_QDC_POSITION_RAMP_DOWN_FRAC	 	((M2_QDC_POSITION_RAMP/M2_SLOW_LOOP_FREQ)*65535)  /* Q16.16 format */
#define M2_QDC_POSITION_CTRL_LIMIT_FRAC		FRAC16(M2_QDC_POSITION_CTRL_LIMIT/M2_N_MAX)
#define M2_QDC_TRAJECTORY_FILTER_FREQ_FRAC  FRAC32(2*PI*M2_QDC_TRAJECTORY_FILTER_FREQ/M2_SLOW_LOOP_FREQ)
#define M2_QDC_POSITION_CTRL_P_GAIN_FRAC	ACC32(M2_QDC_POSITION_CTRL_P_GAIN)

#define M2_SPD_FWD_CUTOFF_FREQ  		   (140.0F)   /* [Hz], cutoff frequency of IIR1 low pass filter */

#define M2_SPD_FWD_IIR_B0				   WARP(M2_SPD_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPD_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPD_FWD_IIR_B1				   WARP(M2_SPD_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_SPD_FWD_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_SPD_FWD_IIR_A1				   (1.0F - M2_SPD_FWD_IIR_B0 - M2_SPD_FWD_IIR_B1)

// Position & Speed Sensors Module - Optical Encoder
//-----------------------------------------------------------------------------------------------------------  

#define M2_QDC_TIMER_PRESCALER				10 //10			 /* Prescaler for the timer within QDC, the prescaling value is 2^Mx_QDC_TIMER_PRESCALER */
#define M2_QDC_CLOCK	 					132000000	 /* [Hz], QDC module clock, which is the bus clock of the system */
#define M2_QDC_SPEED_FILTER_CUTOFF_FREQ 	100.0 		 /* [Hz], cutoff frequency of IIR1 low pass filter for calculated raw speed out of QDC HW feature */
#define M2_QDC_TO_ATT						(0.85F)		 /* attenuation for tracking observer, which is to estimate rotor speed from real QDC position */
#define M2_QDC_TO_FREQ						(300.0)		 /* [Hz], oscillating frequency for tracking observer */

#define M2_QDC_TIMER_FREQUENCY  			(M2_QDC_CLOCK/EXPONENT(2,M2_QDC_TIMER_PRESCALER)) 			/* [Hz], the clock frequency for the timer within QDC */
#define M2_SPEED_CAL_CONST  				((60.0*M2_QDC_TIMER_FREQUENCY/(4*M2_ENCODER_LINES*M2_N_MAX)) * 134217728)	/* A constant to calculate mechanical speed out of QDC HW feature, Q5.27 */
#define M2_QDC_TO_KP_GAIN					(2.0F*M2_QDC_TO_ATT*2*PI*M2_QDC_TO_FREQ)
#define M2_QDC_TO_KI_GAIN					(2*PI*M2_QDC_TO_FREQ * 2*PI*M2_QDC_TO_FREQ/M2_FAST_LOOP_FREQ)
#define M2_QDC_TO_THETA_GAIN				(1.0F/(PI*M2_FAST_LOOP_FREQ))

#define M2_QDC_LINE_RECIPROCAL_4_REV_FRAC_GEN         FRAC32(1.0/(4*M2_ENCODER_LINES)) /* The reciprocal of 4*encoder_lines, in Q1.31 format.
                                                                           This is to expand the raw counter value into range 0~0x7fffffff, which is [0~1) in Q1.31 format  */
#define M2_QDC_LINE_RECIPROCAL_4_POS_GEN              ((0xffffffffU/(4.0*M2_ENCODER_LINES))*1024) /* This constant turns 4*encoder_lines into range 0~0xffffffff in Q22.10 format */

#define M2_QDC_SPEED_FILTER_IIR_B0				   	   WARP(M2_QDC_SPEED_FILTER_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_QDC_SPEED_FILTER_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_QDC_SPEED_FILTER_IIR_B1				       WARP(M2_QDC_SPEED_FILTER_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ)/(WARP(M2_QDC_SPEED_FILTER_CUTOFF_FREQ, M2_SLOW_LOOP_FREQ) + 2.0F)
#define M2_QDC_SPEED_FILTER_IIR_A1				       (1.0F - M2_QDC_SPEED_FILTER_IIR_B0 - M2_QDC_SPEED_FILTER_IIR_B1)
#define M2_QDC_SPEED_FILTER_IIR_B0_FRAC				   FRAC32(M2_QDC_SPEED_FILTER_IIR_B0/2)
#define M2_QDC_SPEED_FILTER_IIR_B1_FRAC				   FRAC32(M2_QDC_SPEED_FILTER_IIR_B1/2)
#define M2_QDC_SPEED_FILTER_IIR_A1_FRAC				   FRAC32(M2_QDC_SPEED_FILTER_IIR_A1/2)


// Sensorless BEMF DQ Observer 
//-----------------------------------------------------------------------------------------------------------  
#define M2_BEMF_DQ_CLOOP_ATT	(0.85F) 	/* attenuation for DQ observer */
#define M2_BEMF_DQ_CLOOP_FREQ	(300.0F)//(300.0F)	/* [Hz], oscillating frequency for DQ observer */
#define M2_BEMF_DQ_TO_ATT		(0.85F)		/* attenuation for tracking observer */
#define M2_BEMF_DQ_TO_FREQ		(70.0)		/* [Hz], oscillating frequency for tracking observer */
#define M2_BEMF_DQ_TO_SPEED_CUTOFF_FREQ  	(637.0F)  /* [Hz], cutoff frequency of IIR1 low pass filter which is for the estimated speed */

#define M2_I_SCALE				(M2_LD/(M2_LD+(M2_R/M2_FAST_LOOP_FREQ)))
#define M2_U_SCALE				((1.0F/M2_FAST_LOOP_FREQ)/(M2_LD+(M2_R/M2_FAST_LOOP_FREQ)))
#define M2_E_SCALE				((1.0F/M2_FAST_LOOP_FREQ)/(M2_LD+(M2_R/M2_FAST_LOOP_FREQ)))
#define M2_WI_SCALE				((M2_LQ/M2_FAST_LOOP_FREQ)/(M2_LD+(M2_R/M2_FAST_LOOP_FREQ)))
#define M2_BEMF_DQ_KP_GAIN 		(2.0F*M2_BEMF_DQ_CLOOP_ATT*2*PI*M2_BEMF_DQ_CLOOP_FREQ*M2_LD - M2_R)
#define M2_BEMF_DQ_KI_GAIN 		(2*PI*M2_BEMF_DQ_CLOOP_FREQ * 2*PI*M2_BEMF_DQ_CLOOP_FREQ *M2_LD/M2_FAST_LOOP_FREQ)
#define M2_BEMF_DQ_TO_KP_GAIN			(2.0F*M2_BEMF_DQ_TO_ATT*2*PI*M2_BEMF_DQ_TO_FREQ)
#define M2_BEMF_DQ_TO_KI_GAIN			(2*PI*M2_BEMF_DQ_TO_FREQ * 2*PI*M2_BEMF_DQ_TO_FREQ/M2_FAST_LOOP_FREQ)
#define M2_BEMF_DQ_TO_THETA_GAIN		(1.0F/(PI*M2_FAST_LOOP_FREQ))

#define M2_TO_SPEED_IIR_B0		WARP(M2_BEMF_DQ_TO_SPEED_CUTOFF_FREQ, M2_FAST_LOOP_FREQ)/(WARP(M2_BEMF_DQ_TO_SPEED_CUTOFF_FREQ, M2_FAST_LOOP_FREQ) + 2.0F)
#define M2_TO_SPEED_IIR_B1		WARP(M2_BEMF_DQ_TO_SPEED_CUTOFF_FREQ, M2_FAST_LOOP_FREQ)/(WARP(M2_BEMF_DQ_TO_SPEED_CUTOFF_FREQ, M2_FAST_LOOP_FREQ) + 2.0F)
#define M2_TO_SPEED_IIR_A1		(1.0F - M2_TO_SPEED_IIR_B0 - M2_TO_SPEED_IIR_B1)

// OpenLoop startup
//-----------------------------------------------------------------------------------------------------------
#define M2_OL_START_SPEED_RAMP		(1000.0F) /* [RPM/s], (mechanical) speed accelerating rate during startup */
#define M2_OL_START_I 				(1.5F)   /* [A], current amplitude during startup */
#define M2_OL_MERG_SPEED			(400.0F)  /* [RPM], mechanical speed when motor merges from startup to closed-loop */
#define M2_MERG_TIME				(14.0)    /* [ms], merging duration when motor reaches speed  Mx_OL_MERG_SPEED。 Merging coefficient increases from 0 to 1 in this phase */

#define M2_OL_START_SPEED_RAMP_RAD	(2.0F*PI*M2_OL_START_SPEED_RAMP*M2_MOTOR_PP/60.0) /* transfer mechanical speed in RPM to electrical angular speed */
#define M2_OL_START_RAMP_INC		(M2_OL_START_SPEED_RAMP_RAD/M2_FAST_LOOP_FREQ)
#define M2_MERG_SPEED_TRH			(2.0F*PI*M2_OL_MERG_SPEED*M2_MOTOR_PP/60.0)
#define M2_MERG_COEFF 				FRAC16(1000.0/(M2_MERG_TIME * M2_FAST_LOOP_FREQ))
  
// Control Structure Module - Scalar Control 
//-----------------------------------------------------------------------------------------------------------  
#define M2_SCALAR_VHZ_FACTOR_GAIN  	(0.09F)  				 /* [v/Hz], voltage against electrical frequency */
#define M2_SCALAR_FREQ_RAMP			(15.0F)			     	 /* [Hz/s], given frequency(motor electrical speed) increases in this rate */

#define M2_SCALAR_INTEG_GAIN		ACC32(2.0*M2_FREQ_MAX/M2_FAST_LOOP_FREQ) /* Integral gain for position generation. Position_frac += M2_SCALAR_INTEG_GAIN * given_freq_frac */
#define M2_SCALAR_RAMP_UP			(M2_SCALAR_FREQ_RAMP/M2_FAST_LOOP_FREQ)
#define M2_SCALAR_RAMP_DOWN			(M2_SCALAR_FREQ_RAMP/M2_FAST_LOOP_FREQ)

// Tamagawa absolute encoder
//-----------------------------------------------------------------------------------------------------------  
#define M2_TAMAGAWA_POSITION_CNT_RESOLUTION (17)  /* Resolution of position counter (one turn), in unit of bits */
#define M2_TAMAGAWA_COM_BAUDRATE            (2500000) /* UART baud rate */
#define M2_TAMAGAWA_REC_DMA_CHANNEL         (2)
#define M2_TAMAGAWA_IF_USE_ABS_POSITION     1 /* TRUE:Use position counter value directly without initial position detection  */
#define M2_TAMAGAWA_ABS_POSITION_COMP       FRAC32(-36.614/180.0)
#define M2_TAMAGAWA_SPEED_CAL_FREQ           M2_SLOW_LOOP_FREQ*2
#define M2_TAMAGAWA_SPEED_CAL_CONST         ((M2_TAMAGAWA_SPEED_CAL_FREQ*30.0/M2_N_MAX)*8388608)  /* Q9.23 format */

#define M2_ABS_TO_ATT						(1.0F)		 /* attenuation for tracking observer, which is to estimate rotor speed from real QDC position */
#define M2_ABS_TO_FREQ						(600.0)		 /* [Hz], oscillating frequency for tracking observer */

#define M2_ABS_TO_KP_GAIN					(2.0F*M1_ABS_TO_ATT*2*PI*M1_ABS_TO_FREQ)
#define M2_ABS_TO_KI_GAIN					(2*PI*M1_ABS_TO_FREQ * 2*PI*M1_ABS_TO_FREQ/M2_PWM_FREQ)
#define M2_ABS_TO_THETA_GAIN				(1.0F/(PI*M2_PWM_FREQ))


// Sinc filter configuration
//-----------------------------------------------------------------------------------------------------------
#define M2_SINC_ORDER                      3
#define M2_SINC_DECIMATION_RATE            64           /* Decimation rate */
#define M2_SINC_IA_CHN                     1             /* Channel number for Ia */
#define M2_SINC_IB_CHN                     2             /* Channel number for Ib */
#define M2_SINC_IC_CHN                     3             /* Channel number for Ic */
#define M2_SINC_UDC_CHN                    0             /* Channel number for Udc. Not used here, must be disabled after init */
#define M2_SINC_I_SCALE                    32.0          /* [A], the current that corresponds to the maximum measurable range of the external SigmaDelta ADC. E.g. the input range of ADC is
                                                         -320mV ~ 320mV, and the shunt resistor is 10mOhm, the measurable current range is 32A */
#define M2_SINC_U_SCALE                    82.76         /* [v], the DC bus voltage that corresponds to the maximum measurable range of the external SigmaDelta ADC. */
#define M2_SINC_RSLT_RSHIFT_BITS           6             /* The maximum possible result is 24bit, this field defines how many lower bits are ignored */
#define M2_SINC_MODULE_CLOCK_PRESCALE      1             /* Sinc filter clock frequency is M2_SINC_MODULE_CLOCK_FREQ/(M2_SINC_MODULE_CLOCK_DIV+1)/(2^M2_SINC_MODULE_CLOCK_PRESCALE) */
#define M2_SINC_MODULE_CLOCK_DIV           4             /* Must be >= 2. Sinc filter clock frequency is M2_SINC_MODULE_CLOCK_FREQ/(M2_SINC_MODULE_CLOCK_DIV+1)/(2^M2_SINC_MODULE_CLOCK_PRESCALE) */


#define M2_SINC_MODULE_CLOCK_FREQ          132000000.0   /* [Hz], sinc filter module clock */
#define M2_SINC_OUTPUT_CLOCK_FREQ          (M2_SINC_MODULE_CLOCK_FREQ/(M2_SINC_MODULE_CLOCK_DIV*EXPONENT(2,M2_SINC_MODULE_CLOCK_PRESCALE)))
#define M2_SINC_DECIMATION_CLOCK_FREQ      (M2_SINC_OUTPUT_CLOCK_FREQ/M2_SINC_DECIMATION_RATE) /* [Hz] */
#define M2_SINC_FIFO_DEPTH                 (M2_SINC_DECIMATION_CLOCK_FREQ/M2_PWM_FREQ) /* Must be an integer! */
#define M2_SINC_RSLT_SCALE                 EXPONENT(M2_SINC_DECIMATION_RATE, M2_SINC_ORDER)/EXPONENT(2,M2_SINC_RSLT_RSHIFT_BITS) /* Maximum result value after the right shift */

#endif

//End of generated file
/**********************************************************************/
