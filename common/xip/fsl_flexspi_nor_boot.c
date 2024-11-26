/*
 * Copyright 2021-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flexspi_nor_boot.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.xip_device"
#endif

#if defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".boot_hdr.container"), used))
#elif defined(__ICCARM__)
#pragma location = ".boot_hdr.container"
#endif
/*************************************
 *  Container Data
 *************************************/
const container container_data = {{
                                      CNT_VERSION,
                                      CNT_SIZE,
                                      CNT_TAG_HEADER,
                                      CNT_FLAGS,
                                      CNT_SW_VER,
                                      CNT_FUSE_VER,
                                      CNT_NUM_IMG,
                                      sizeof(cnt_hdr) + CNT_NUM_IMG * sizeof(image_entry),
                                      0,
                                  },
                                  {{(uint32_t)app_image_offset,
                                    IMAGE_SIZE,
                                    IMAGE_LOAD_ADDRESS,
                                    0x00000000,
                                    IMAGE_LOAD_ADDRESS,
                                    0x00000000,
                                    IMG_FLAGS,
                                    0x0,
                                    {0},
                                    {0}}},
                                  {
                                      SGNBK_VERSION,
                                      SGNBK_SIZE,
                                      SGNBK_TAG,
                                      0x0,
                                      0x0,
                                      0x0,
                                      0x0,
                                  }};

#endif

#if defined(__MCUXPRESSO) && defined(XIP_BOOT_HEADER_ENABLE) && (XIP_BOOT_HEADER_ENABLE == 1)
/*************************************
 *  XMCD Data
 *************************************/

#if defined(USE_HYPERRAM)
__attribute__((section(".boot_hdr.xmcd_data"), used)) const uint32_t xmcd_data[] = {
#if (defined(CPU_MIMXRT1187AVM8B_cm7) || defined(CPU_MIMXRT1187AVM8B_cm33))
    0
#else
    0xC002000C, /* FlexSPI instance 2 */
    0xC1000800, /* Option words = 2 */
    0x00010000  /* PINMUX Secondary group */
#endif
};
#endif

#if defined(USE_SDRAM)
__attribute__((section(".boot_hdr.xmcd_data"), used)) const uint32_t xmcd_data[] = {
#if (defined(CPU_MIMXRT1187AVM8B_cm7) || defined(CPU_MIMXRT1187AVM8B_cm33))
    0
#else
    0xC010000D, /* SEMC -> SDRAM */
    0xA60001A1, /* SDRAM config */
    0x00008000, /* SDRAM config */
    0X00000001  /* SDRAM config */
#endif
};
#endif

#endif
