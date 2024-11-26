/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "crc.h"

uint8_t crc8(uint8_t *ptr, uint16_t u16Size)
{
#if TRANSPOSE_BIT_OUT == 1
	uint8_t u8Tmp = 0;
#endif
	uint8_t u8Crc = INIT_SEED;
	uint8_t u8Poly = POLYNOMIAL;
	uint16_t i,j;
	
	for(i=0; i<u16Size; i++)
	{
#if TRANSPOSE_BIT_IN == 1
		u8Crc ^= ((ptr[i]&0x80)>>7);
		u8Crc ^= ((ptr[i]&0x40)>>5);
		u8Crc ^= ((ptr[i]&0x20)>>3);
		u8Crc ^= ((ptr[i]&0x10)>>1);
		u8Crc ^= ((ptr[i]&0x08)<<1);
		u8Crc ^= ((ptr[i]&0x04)<<3);
		u8Crc ^= ((ptr[i]&0x02)<<5);
		u8Crc ^= ((ptr[i]&0x01)<<7);
		
#else
		u8Crc ^= ptr[i];
#endif
		
		for(j=0; j<8; j++)
		{
			if((u8Crc & 0x80) != 0)
			{

				u8Crc = (uint8_t)((u8Crc<<1)^u8Poly);
		
			}
			else
			{
				u8Crc <<= 1;
			}
		}
	}
#if XOR_OUT == 1
	u8Crc ^= 0xFF;
#endif

#if TRANSPOSE_BIT_OUT == 1
	u8Tmp |= ((u8Crc&0x80)>>7);
	u8Tmp |= ((u8Crc&0x40)>>5);
	u8Tmp |= ((u8Crc&0x20)>>3);
	u8Tmp |= ((u8Crc&0x10)>>1);
	u8Tmp |= ((u8Crc&0x08)<<1);
	u8Tmp |= ((u8Crc&0x04)<<3);
	u8Tmp |= ((u8Crc&0x02)<<5);
	u8Tmp |= ((u8Crc&0x01)<<7);
	u8Crc = u8Tmp;
	
#endif
	return u8Crc;
}


