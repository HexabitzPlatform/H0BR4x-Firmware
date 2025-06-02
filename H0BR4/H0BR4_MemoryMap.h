/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H0BR4_MemoryMap.h
 Description: Defines STM32G0B0 memory map.
 Flash: Application (500 KB), topology (2 KB), snippets (2 KB), EEPROM (8 KB, 1000 16-bit variables).
 */

/* Define to prevent recursive inclusion************************************/
#ifndef H0BR4_MEMORYMAP_H
#define H0BR4_MEMORYMAP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Memory map: - STM32G0B0 : 0x08000000 - 0x0807FFFF >> 512 KB divided into :
 * Application: 0x08000000 - 0x0807A7FF >> 500 KB.
 * Topology Address (RO): 0x0807A800 - 0x0807AFFF >> 2 KB, used to store topology information.
 * Snippets Address (RO): 0x0807B000 - 0x0807B7FF >> 2 KB, used to store Command Snippet.
 * Emulated EEPROM Address: 0x0807B800 - 0x0807F800 >> 8 KB,
 * fits 1000 16-bit variables in 8 pages (4 basic pages + 4 backup pages).
 */
#define APP_START_ADDRESS  		((uint32_t)0x08000000)
#define TOPOLOGY_START_ADDRESS  ((uint32_t)0x0807A800)    /* topology is stored here */
#define SNIPPETS_START_ADDRESS  ((uint32_t)0x0807B000)    /* Snippet are stored here */
#define EEPROM_START_ADDRESS  	((uint32_t)0x0807B800)    /* EE_Variables are stored here */
#define TOPOLOGY_PAGE_NUM		373
#define SNIPPETS_PAGE_NUM		374

#ifdef __cplusplus
}
#endif

#endif /* H0BR4_MEMORYMAP_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
