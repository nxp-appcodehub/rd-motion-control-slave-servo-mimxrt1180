/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MIMXRT1189xxxxx
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.8.1 [Build 1197] [2023-10-27] on Oct 8, 2024, 3:12:38 PM
 */

MEMORY
{
  /* Define each memory region */
  BOARD_FLASH (rx) : ORIGIN = 0x28000000, LENGTH = 0x1000000 /* 16M bytes (alias Flash) */  
  SRAM_ITC_cm33 (rwx) : ORIGIN = 0xffe0000, LENGTH = 0x20000 /* 128K bytes (alias RAM) */  
  SRAM_DTC_cm33 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000 /* 128K bytes (alias RAM2) */  
  SHMEM_REGION (rwx) : ORIGIN = 0x20500000, LENGTH = 0x40000 /* 256K bytes (alias RAM3) */  
  M7_ITCM_ALIAS (rwx) : ORIGIN = 0x203c0000, LENGTH = 0x40000 /* 256K bytes (alias RAM4) */  
}

  /* Define a symbol for the top of each memory region */
  __base_BOARD_FLASH = 0x28000000  ; /* BOARD_FLASH */  
  __base_Flash = 0x28000000 ; /* Flash */  
  __top_BOARD_FLASH = 0x28000000 + 0x1000000 ; /* 16M bytes */  
  __top_Flash = 0x28000000 + 0x1000000 ; /* 16M bytes */  
  __base_SRAM_ITC_cm33 = 0xffe0000  ; /* SRAM_ITC_cm33 */  
  __base_RAM = 0xffe0000 ; /* RAM */  
  __top_SRAM_ITC_cm33 = 0xffe0000 + 0x20000 ; /* 128K bytes */  
  __top_RAM = 0xffe0000 + 0x20000 ; /* 128K bytes */  
  __base_SRAM_DTC_cm33 = 0x20000000  ; /* SRAM_DTC_cm33 */  
  __base_RAM2 = 0x20000000 ; /* RAM2 */  
  __top_SRAM_DTC_cm33 = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __top_RAM2 = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __base_SHMEM_REGION = 0x20500000  ; /* SHMEM_REGION */  
  __base_RAM3 = 0x20500000 ; /* RAM3 */  
  __top_SHMEM_REGION = 0x20500000 + 0x40000 ; /* 256K bytes */  
  __top_RAM3 = 0x20500000 + 0x40000 ; /* 256K bytes */  
  __base_M7_ITCM_ALIAS = 0x203c0000  ; /* M7_ITCM_ALIAS */  
  __base_RAM4 = 0x203c0000 ; /* RAM4 */  
  __top_M7_ITCM_ALIAS = 0x203c0000 + 0x40000 ; /* 256K bytes */  
  __top_RAM4 = 0x203c0000 + 0x40000 ; /* 256K bytes */  
