/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CRC_H_
#define CRC_H_
#include <stdint.h>

#define INIT_SEED 0x00
#define POLYNOMIAL 0x01
#define XOR_OUT	          0  // 1 means XOR the final output CRC value
#define TRANSPOSE_BIT_IN  1  // 1 means transpose input bits
#define TRANSPOSE_BIT_OUT 1  // 1 means transpose final output CRC value


extern uint8_t crc8(uint8_t *ptr, uint16_t u16Size);

#endif /* CRC_H_ */
