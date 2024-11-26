/*
    * Copyright 2022 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef MOTOR_CONTROL_TASK_TEKNIC_TRIGONOMETRIC_H
#define MOTOR_CONTROL_TASK_TEKNIC_TRIGONOMETRIC_H

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PI 3.1415926
#define EXPONENT_1(x)	(x)
#define EXPONENT_3(x)	(x*x*x)
#define EXPONENT_5(x)	(x*x*x*x*x)
#define EXPONENT_7(x)	(x*x*x*x*x*x*x)
#define EXPONENT_9(x)	(x*x*x*x*x*x*x*x*x)
#define EXPONENT_11(x)	(x*x*x*x*x*x*x*x*x*x*x)

#define EXPONENT_0(x)   (1)
#define EXPONENT_2(x)	(x*x)
#define EXPONENT_4(x)	(x*x*x*x)
#define EXPONENT_6(x)	(x*x*x*x*x*x)
#define EXPONENT_8(x)	(x*x*x*x*x*x*x*x)
#define EXPONENT_10(x)	(x*x*x*x*x*x*x*x*x*x)
#define EXPONENT_12(x)	(x*x*x*x*x*x*x*x*x*x*x*x)
#define EXPONENT_13(x)	(x*x*x*x*x*x*x*x*x*x*x*x*x)
#define EXPONENT_14(x)	(x*x*x*x*x*x*x*x*x*x*x*x*x*x)
#define EXPONENT_15(x)	(x*x*x*x*x*x*x*x*x*x*x*x*x*x*x)
#define EXPONENT(x,y)  EXPONENT_intermediate(x,y)
#define EXPONENT_intermediate(x,y)  EXPONENT_##y(x)  // x^y

#define TAN(x) 		(x+EXPONENT_3(x)/3.0F+2*EXPONENT_5(x)/15.0F+17*EXPONENT_7(x)/315.0F+62*EXPONENT_9(x)/2835.0F+1382*EXPONENT_11(x)/155925.0F)

#define WARP(x,y)		(2*TAN(2*PI*x/(2*y))) // x is cutoff frequency in Hz. y is fast loop frequency in Hz. (y/x) must be larger than 2

#endif /* MOTOR_CONTROL_TASK_TEKNIC_TRIGONOMETRIC_H */
