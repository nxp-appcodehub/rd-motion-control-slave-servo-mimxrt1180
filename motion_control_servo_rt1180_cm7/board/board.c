/*
 * Copyright 2021-2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"
#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
#include "fsl_lpi2c.h"
#endif /* SDK_I2C_BASED_COMPONENT_USED */
#if defined(SDK_NETC_USED) && SDK_NETC_USED
#include "fsl_netc_soc.h"
#endif /* SDK_NETC_USED */
#include "fsl_iomuxc.h"
#include "fsl_cache.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_FLEXSPI_DLL_LOCK_RETRY (10)

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Get debug console frequency. */
uint32_t BOARD_DebugConsoleSrcFreq(void)
{
    return CLOCK_GetRootClockFreq(BOARD_DEBUG_UART_CLK_ROOT);
}

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq = BOARD_DebugConsoleSrcFreq();
    DbgConsole_Init(BOARD_DEBUG_UART_INSTANCE, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

#if defined(SDK_I2C_BASED_COMPONENT_USED) && SDK_I2C_BASED_COMPONENT_USED
void BOARD_LPI2C_Init(LPI2C_Type *base, uint32_t clkSrc_Hz)
{
    lpi2c_master_config_t lpi2cConfig = {0};

    /*
     * lpi2cConfig.debugEnable = false;
     * lpi2cConfig.ignoreAck = false;
     * lpi2cConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * lpi2cConfig.baudRate_Hz = 100000U;
     * lpi2cConfig.busIdleTimeout_ns = 0;
     * lpi2cConfig.pinLowTimeout_ns = 0;
     * lpi2cConfig.sdaGlitchFilterWidth_ns = 0;
     * lpi2cConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&lpi2cConfig);
    LPI2C_MasterInit(base, &lpi2cConfig, clkSrc_Hz);
}

status_t BOARD_LPI2C_Send(LPI2C_Type *base,
                          uint8_t deviceAddress,
                          uint32_t subAddress,
                          uint8_t subAddressSize,
                          uint8_t *txBuff,
                          uint8_t txBuffSize)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = txBuff;
    xfer.dataSize       = txBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

status_t BOARD_LPI2C_Receive(LPI2C_Type *base,
                             uint8_t deviceAddress,
                             uint32_t subAddress,
                             uint8_t subAddressSize,
                             uint8_t *rxBuff,
                             uint8_t rxBuffSize)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Read;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = rxBuff;
    xfer.dataSize       = rxBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

status_t BOARD_LPI2C_SendSCCB(LPI2C_Type *base,
                              uint8_t deviceAddress,
                              uint32_t subAddress,
                              uint8_t subAddressSize,
                              uint8_t *txBuff,
                              uint8_t txBuffSize)
{
    lpi2c_master_transfer_t xfer;

    xfer.flags          = kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = txBuff;
    xfer.dataSize       = txBuffSize;

    return LPI2C_MasterTransferBlocking(base, &xfer);
}

status_t BOARD_LPI2C_ReceiveSCCB(LPI2C_Type *base,
                                 uint8_t deviceAddress,
                                 uint32_t subAddress,
                                 uint8_t subAddressSize,
                                 uint8_t *rxBuff,
                                 uint8_t rxBuffSize)
{
    status_t status;
    lpi2c_master_transfer_t xfer;

    xfer.flags          = kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = deviceAddress;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = subAddress;
    xfer.subaddressSize = subAddressSize;
    xfer.data           = NULL;
    xfer.dataSize       = 0;

    status = LPI2C_MasterTransferBlocking(base, &xfer);

    if (kStatus_Success == status)
    {
        xfer.subaddressSize = 0;
        xfer.direction      = kLPI2C_Read;
        xfer.data           = rxBuff;
        xfer.dataSize       = rxBuffSize;

        status = LPI2C_MasterTransferBlocking(base, &xfer);
    }

    return status;
}

void BOARD_Accel_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_ACCEL_I2C_BASEADDR, BOARD_ACCEL_I2C_CLOCK_FREQ);
}

status_t BOARD_Accel_I2C_Send(uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint32_t txBuff)
{
    uint8_t data = (uint8_t)txBuff;

    return BOARD_LPI2C_Send(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subaddressSize, &data, 1);
}

status_t BOARD_Accel_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subaddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_LPI2C_Receive(BOARD_ACCEL_I2C_BASEADDR, deviceAddress, subAddress, subaddressSize, rxBuff, rxBuffSize);
}

void BOARD_Codec_I2C_Init(void)
{
    BOARD_LPI2C_Init(BOARD_CODEC_I2C_BASEADDR, BOARD_CODEC_I2C_CLOCK_FREQ);
}

status_t BOARD_Codec_I2C_Send(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
    return BOARD_LPI2C_Send(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, (uint8_t *)txBuff,
                            txBuffSize);
}

status_t BOARD_Codec_I2C_Receive(
    uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    return BOARD_LPI2C_Receive(BOARD_CODEC_I2C_BASEADDR, deviceAddress, subAddress, subAddressSize, rxBuff, rxBuffSize);
}
#endif /* SDK_I2C_BASED_COMPONENT_USED */

/* MPU configuration. */
#if __CORTEX_M == 7
void BOARD_ConfigMPU(void)
{
    volatile uint32_t i;

    /* Disable I cache and D cache */
    L1CACHE_DisableICache();
    L1CACHE_DisableDCache();

    /* Disable MPU */
    ARM_MPU_Disable();

    /* clang-format off */

    /* MPU configure:
     * Use ARM_MPU_RASR(DisableExec, AccessPermission, TypeExtField, IsShareable, IsCacheable, IsBufferable, SubRegionDisable, Size)
     * API in mpu_armv7.h.
     * param DisableExec       Instruction access (XN) disable bit,0=instruction fetches enabled, 1=instruction fetches disabled.
     * param AccessPermission  Data access permissions, allows you to configure read/write access for User and Privileged mode.
     *                         Use MACROS defined in mpu_armv7.h:
     *                         ARM_MPU_AP_NONE/ARM_MPU_AP_PRIV/ARM_MPU_AP_URO/ARM_MPU_AP_FULL/ARM_MPU_AP_PRO/ARM_MPU_AP_RO
     *
     * Combine TypeExtField/IsShareable/IsCacheable/IsBufferable to configure MPU memory access attributes.
     *  TypeExtField  IsShareable  IsCacheable  IsBufferable   Memory Attribute    Shareability        Cache
     *     0             x           0           0             Strongly Ordered    shareable
     *     0             x           0           1              Device             shareable
     *     0             0           1           0              Normal             not shareable   Outer and inner write
     *                                                                                             through no write allocate
     *     0             0           1           1              Normal             not shareable   Outer and inner write
     *                                                                                             back no write allocate
     *     0             1           1           0              Normal             shareable       Outer and inner write
     *                                                                                             through no write allocate
     *     0             1           1           1              Normal             shareable       Outer and inner write
     *                                                                                             back no write allocate
     *     1             0           0           0              Normal             not shareable   outer and inner
     *                                                                                             noncache
     *     1             1           0           0              Normal             shareable       outer and inner
     *                                                                                             noncache
     *     1             0           1           1              Normal             not shareable   outer and inner write
     *                                                                                             back write/read acllocate
     *     1             1           1           1              Normal             shareable       outer and inner write
     *                                                                                             back write/read acllocate
     *     2             x           0           0              Device             not shareable
     *   Above are normal use settings, if your want to see more details or want to config different inner/outter cache
     * policy, please refer to Table 4-55 /4-56 in arm cortex-M7 generic user guide <dui0646b_cortex_m7_dgug.pdf>
     *
     * param SubRegionDisable  Sub-region disable field. 0=sub-region is enabled, 1=sub-region is disabled.
     * param Size              Region size of the region to be configured. use ARM_MPU_REGION_SIZE_xxx MACRO in mpu_armv7.h.
     */

    /* clang-format on */

    /*
     * Add default region to deny access to whole address space to workaround speculative prefetch.
     * Refer to Arm errata 1013783-B for more details.
     */

    /* Region 0 setting: Instruction access disabled, No data access permission. */
    MPU->RBAR = ARM_MPU_RBAR(0, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(1, ARM_MPU_AP_NONE, 0, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_4GB);

    /* Region 3 setting: Memory with Device type, not shareable, non-cacheable. */
    MPU->RBAR = ARM_MPU_RBAR(1, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 2, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_2GB);

    /* Region 4 setting: Memory with Normal type, not shareable, outer/inner write back */ /*ITCM*/
    MPU->RBAR = ARM_MPU_RBAR(2, 0x00000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

    /* Region 5 setting: Memory with Normal type, not shareable, outer/inner write back */ /*DTCM*/
    MPU->RBAR = ARM_MPU_RBAR(3, 0x20000000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_256KB);

    /* Region 6 setting: Memory with Normal type, not shareable, outer/inner write back */ /*OCRAM1*/
    MPU->RBAR = ARM_MPU_RBAR(4, 0x20480000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_512KB);

    /* Region 7 setting: Memory with Normal type, not shareable, non-cacheable */ /*OCRAM2*/
    MPU->RBAR = ARM_MPU_RBAR(5, 0x20500000U);
    MPU->RASR = ARM_MPU_RASR(0, ARM_MPU_AP_FULL, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_256KB);

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_HFNMIENA_Msk);

    /* Enable I cache and D cache */
    L1CACHE_EnableDCache();
    L1CACHE_EnableICache();
}
#elif __CORTEX_M == 33
void BOARD_ConfigMPU(void)
{
    uint32_t i;
    uint8_t attr;

    /* Disable code & system cache */
    XCACHE_DisableCache(XCACHE_PC);
    XCACHE_DisableCache(XCACHE_PS);

    /* Disable MPU */
    ARM_MPU_Disable();

    /* Attr0: device. */
    ARM_MPU_SetMemAttr(0U, ARM_MPU_ATTR(ARM_MPU_ATTR_DEVICE, ARM_MPU_ATTR_DEVICE));

    /* Attr1: non cacheable. */
    ARM_MPU_SetMemAttr(1U, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));

    /* Attr2: non transient, write through, read allocate. */
    attr = ARM_MPU_ATTR_MEMORY_(0U, 0U, 1U, 0U);
    ARM_MPU_SetMemAttr(2U, ARM_MPU_ATTR(attr, attr));

    /* Attr3: non transient, write back, read/write allocate. */
    attr = ARM_MPU_ATTR_MEMORY_(0U, 1U, 1U, 1U);
    ARM_MPU_SetMemAttr(3U, ARM_MPU_ATTR(attr, attr));

    /* NOTE:
     *   1. When memory regions overlap, the processor generates a fault if a core access hits the overlapping regions
     */

    /* Region 1 (Code TCM): [0x0FFE0000, 0x0FFFFFFF, 128K] */
    /* non-shareable, read/write in privilege and non-privilege, executable. Attr 2 */
    ARM_MPU_SetRegion(1U, ARM_MPU_RBAR(0x0FFE0000, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x0FFFFFFF, 2U));

    /* Region 2 (System TCM): [0x20000000, 0x2001FFFF, 128K] */
    /* non-shareable, read/write in privilege and non-privilege, executable. Attr 3 */
    ARM_MPU_SetRegion(2U, ARM_MPU_RBAR(0x20000000, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x2001FFFF, 3U));

    /* Region 3 (CM7 I/D TCM): [0x203C0000, 0x2043FFFF, 512K] */
    /* non-shareable, read/write in privilege and non-privilege, execute-never. Attr 1 (non cacheable). */
    ARM_MPU_SetRegion(3U, ARM_MPU_RBAR(0x203C0000, ARM_MPU_SH_NON, 0U, 1U, 1U), ARM_MPU_RLAR(0x2043FFFF, 1U));

    /* Region 4 (CM7 I/D TCM): [0x303C0000, 0x3043FFFF, 512K] */
    /* non-shareable, read/write in privilege and non-privilege, execute-never. Attr 1 (non cacheable). */
    ARM_MPU_SetRegion(4U, ARM_MPU_RBAR(0x303C0000, ARM_MPU_SH_NON, 0U, 1U, 1U), ARM_MPU_RLAR(0x3043FFFF, 1U));

    /*
       As common setting, not set this region to avoid potential overlapping setting with NCACHE(region 8)
       and SHMEM(region 9) for specific build configuration, but use the default attribute of the background
       system address map.
       The default attribute of the background system address map for this address space:
         normal, write back, write/read allocate, non-shareable, read/write in privilege and non-privilege, executable

       If application needs to fine tune MPU settings, here is an example:
       // Region 4 (OCRAM): [0x20480000, 0x2053FFFF, 768K]
       // non-shareable, read/write in privilege and non-privilege, executable. Attr 3
       ARM_MPU_SetRegion(4U, ARM_MPU_RBAR(0x20480000, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x2053FFFF, 3U));
    */

    /* Region 5 (FlexSPI1, Nor Flash): [0x28000000, 0x28FFFFFF, 16M] */
    /* non-shareable, read only in privilege and non-privileged, executable. Attr 2 */
    ARM_MPU_SetRegion(5U, ARM_MPU_RBAR(0x28000000, ARM_MPU_SH_NON, 1U, 1U, 0U), ARM_MPU_RLAR(0x28FFFFFF, 2U));

    /* Region 6 (Peripherals): [0x40000000, 0x7FFFFFFF, 1G ] */
    /* non-shareable, read/write in privilege and non-privileged, execute-never. Attr 0 (device). */
    ARM_MPU_SetRegion(6U, ARM_MPU_RBAR(0x40000000, ARM_MPU_SH_NON, 0U, 1U, 1U), ARM_MPU_RLAR(0x7FFFFFFF, 0U));

    /* Region 7 (OCRAM2): [0x20500000, 0x2053FFFF, 256K] */
    /* non-shareable, read/write in privilege and non-privilege, execute-never. Attr 1 (non cacheable). */
    ARM_MPU_SetRegion(7U, ARM_MPU_RBAR(0x20500000, ARM_MPU_SH_NON, 0U, 1U, 0U), ARM_MPU_RLAR(0x2053FFFF, 1U));

    /* Enable MPU */
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);

    /* Enable code & system cache */
    XCACHE_EnableCache(XCACHE_PS);
    XCACHE_EnableCache(XCACHE_PC);
}
#endif

void BOARD_SD_Pin_Config(uint32_t speed, uint32_t strength)
{
}

void BOARD_MMC_Pin_Config(uint32_t speed, uint32_t strength)
{
}

void BOARD_DeinitFlash(FLEXSPI_Type *base)
{
#if (__CORTEX_M == 7)
    if (SCB_CCR_IC_Msk == (SCB_CCR_IC_Msk & SCB->CCR))
    {
        SCB_DisableICache();
    }
#endif

#if (__CORTEX_M == 33)
    if ((XCACHE_PC->CCR & XCACHE_CCR_ENCACHE_MASK) == 1U) /* disabled if enabled */
    {
        /* Enable the to push all modified lines. */
        XCACHE_PC->CCR |= XCACHE_CCR_PUSHW0_MASK | XCACHE_CCR_PUSHW1_MASK | XCACHE_CCR_GO_MASK;
        /* Wait until the cache command completes. */
        while ((XCACHE_PC->CCR & XCACHE_CCR_GO_MASK) != 0x00U)
        {
        }
        /* As a precaution clear the bits to avoid inadvertently re-running this command. */
        XCACHE_PC->CCR &= ~(XCACHE_CCR_PUSHW0_MASK | XCACHE_CCR_PUSHW1_MASK);
        XCACHE_PC->CCR &= ~XCACHE_CCR_ENCACHE_MASK;
        __ISB();
        __DSB();
    }
#endif

    /* Enable FLEXSPI module */
    base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    /* Wait until FLEXSPI is not busy */
    while (!((base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) && (base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK)))
    {
    }
    /* Disable module during the reset procedure */
    base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
}

void BOARD_InitFlash(FLEXSPI_Type *base)
{
    uint32_t status;
    uint32_t lastStatus;
    uint32_t retry;

    /* If serial root clock is >= 100 MHz, DLLEN set to 1, OVRDEN set to 0, then SLVDLYTARGET setting of 0x0 is
     * recommended. */
    base->DLLCR[0] = 0x1U;

    /* Enable FLEXSPI module */
    base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

    base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    while (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
    {
    }

    /* Need to wait DLL locked if DLL enabled */
    if (0U != (base->DLLCR[0] & FLEXSPI_DLLCR_DLLEN_MASK))
    {
        lastStatus = base->STS2;
        retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
        /* Wait slave delay line locked and slave reference delay line locked. */
        do
        {
            status = base->STS2;
            if ((status & (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK)) ==
                (FLEXSPI_STS2_AREFLOCK_MASK | FLEXSPI_STS2_ASLVLOCK_MASK))
            {
                /* Locked */
                retry = 100;
                break;
            }
            else if (status == lastStatus)
            {
                /* Same delay cell number in calibration */
                retry--;
            }
            else
            {
                retry      = BOARD_FLEXSPI_DLL_LOCK_RETRY;
                lastStatus = status;
            }
        } while (retry > 0);
        /* According to ERR011377, need to delay at least 100 NOPs to ensure the DLL is locked. */
        for (; retry > 0U; retry--)
        {
            __NOP();
        }
    }

#if (__CORTEX_M == 7)
    SCB_EnableICache();
#endif

#if (__CORTEX_M == 33)
    if ((XCACHE_PC->CCR & XCACHE_CCR_ENCACHE_MASK) == 0U)
    {
        /* set command to invalidate all ways and write GO bit to initiate command */
        XCACHE_PC->CCR = XCACHE_CCR_INVW1_MASK | XCACHE_CCR_INVW0_MASK;
        XCACHE_PC->CCR |= XCACHE_CCR_GO_MASK;
        /* Wait until the command completes */
        while ((XCACHE_PC->CCR & XCACHE_CCR_GO_MASK) != 0U)
        {
        }
        /* Enable cache */
        XCACHE_PC->CCR = XCACHE_CCR_ENCACHE_MASK;
        __ISB();
        __DSB();
    }
#endif
}

/* BOARD_SetFlexspiClock run in RAM used to configure FlexSPI clock source and divider when XIP. */
void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint8_t src, uint32_t divider)
{
    clock_root_t root;
    clock_lpcg_t lpcg;

    if (base == FLEXSPI1)
    {
        root = kCLOCK_Root_Flexspi1;
        lpcg = kCLOCK_Flexspi1;
    }
    else if (base == FLEXSPI2)
    {
        root = kCLOCK_Root_Flexspi2;
        lpcg = kCLOCK_Flexspi2;
    }
    else
    {
        return;
    }

    if (((CCM->CLOCK_ROOT[root].CONTROL & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) != CCM_CLOCK_ROOT_CONTROL_MUX(src)) ||
        ((CCM->CLOCK_ROOT[root].CONTROL & CCM_CLOCK_ROOT_CONTROL_DIV_MASK) != CCM_CLOCK_ROOT_CONTROL_DIV(divider - 1)))
    {
        /* Always deinit FLEXSPI and init FLEXSPI for the flash to make sure the flash works correctly after the
         FLEXSPI root clock changed as the default FLEXSPI configuration may does not work for the new root clock
         frequency. */
        BOARD_DeinitFlash(base);

        /* Disable clock before changing clock source */
        CCM->LPCG[lpcg].DIRECT &= ~CCM_LPCG_DIRECT_ON_MASK;
        __DSB();
        __ISB();
        while (CCM->LPCG[lpcg].STATUS0 & CCM_LPCG_STATUS0_ON_MASK)
        {
        }

        /* Update flexspi clock. */
        CCM->CLOCK_ROOT[root].CONTROL = CCM_CLOCK_ROOT_CONTROL_MUX(src) | CCM_CLOCK_ROOT_CONTROL_DIV(divider - 1);
        __DSB();
        __ISB();
        (void)CCM->CLOCK_ROOT[root].CONTROL;

        /* Enable FLEXSPI clock again */
        CCM->LPCG[lpcg].DIRECT |= CCM_LPCG_DIRECT_ON_MASK;
        __DSB();
        __ISB();
        while (!(CCM->LPCG[lpcg].STATUS0 & CCM_LPCG_STATUS0_ON_MASK))
        {
        }

        BOARD_InitFlash(base);
    }
}

/* This function is used to change FlexSPI clock to a stable source before clock sources(Such as PLL and Main clock)
 * updating in case XIP(execute code on FLEXSPI memory.) */
void BOARD_FlexspiClockSafeConfig(void)
{
    /* Move FLEXSPI clock source to OSC_RC_24M to avoid instruction/data fetch issue in XIP when updating PLL. */
    BOARD_SetFlexspiClock(FLEXSPI1, 0U, 1U);
}

#if defined(SDK_NETC_USED) && SDK_NETC_USED
void BOARD_NETC_Init(void)
{
    /* EP and Switch port 0 use RMII interface. */
    NETC_SocSetMiiMode(kNETC_SocLinkEp0, kNETC_RmiiMode);
    NETC_SocSetMiiMode(kNETC_SocLinkSwitchPort0, kNETC_RmiiMode);

    /* Switch port 1~3 use RGMII interface. */
    NETC_SocSetMiiMode(kNETC_SocLinkSwitchPort1, kNETC_RgmiiMode);
    NETC_SocSetMiiMode(kNETC_SocLinkSwitchPort2, kNETC_RgmiiMode);
    NETC_SocSetMiiMode(kNETC_SocLinkSwitchPort3, kNETC_RgmiiMode);

    /* Output reference clock for RMII interface. */
    NETC_SocSetRmiiRefClk(kNETC_SocLinkEp0, true);
    NETC_SocSetRmiiRefClk(kNETC_SocLinkSwitchPort0, true);

    /* Unlock the IERB. It will warm reset whole NETC. */
    if (NETC_IERBUnlock() == kStatus_Success)
    {
        while (!NETC_IERBIsUnlockOver())
        {
        }
    }

    /* Set the access attribute, otherwise MSIX access will be blocked. */
    NETC_IERB->ARRAY_NUM_RC[0].RCMSIAMQR &= ~(7U << 27);
    NETC_IERB->ARRAY_NUM_RC[0].RCMSIAMQR |= (1U << 27);

    /* Set PHY address in IERB to use MAC port MDIO, otherwise the access will be blocked. */
    NETC_SocSetLinkAddr(kNETC_SocLinkEp0, BOARD_EP0_PHY_ADDR);
    NETC_SocSetLinkAddr(kNETC_SocLinkSwitchPort0, BOARD_SWT_PORT0_PHY_ADDR);
    NETC_SocSetLinkAddr(kNETC_SocLinkSwitchPort1, BOARD_SWT_PORT1_PHY_ADDR);
    NETC_SocSetLinkAddr(kNETC_SocLinkSwitchPort2, BOARD_SWT_PORT2_PHY_ADDR);
    NETC_SocSetLinkAddr(kNETC_SocLinkSwitchPort3, BOARD_SWT_PORT3_PHY_ADDR);

    /* Lock the IERB. */
    assert(NETC_IERBLock() == kStatus_Success);
    while (!NETC_IERBIsLockOver())
    {
    }
}
#endif /* SDK_NETC_USED */
