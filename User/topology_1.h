/*
    BitzOS (BOS) V0.2.3 - Copyright (C) 2016 Hexabitz
    All rights reserved

    File Name     : topology_1.h
    Description   : Array topology definition.
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __topology_1_H
#define __topology_1_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"


#define __N	2					// Number of array modules

// Array modules
#define _mod1	1<<3
#define _mod2	2<<3

// Topology
static uint16_t array[__N][7] = {
{ _H01R0, 0, 0, _mod2|P5, 0, 0, 0},							// Module 1
{ _H01R0, 0, 0, 0, 0, _mod1|P3, 0},			        // Module 2
};

// Configurations for duplex serial ports
#if ( _module == 1 )
#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P4pol_normal	1
	#define	_P4pol_normal	1
#endif

#if ( _module == 2 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P4pol_reversed	1
	#define	_P4pol_normal	1

#endif

#ifdef __cplusplus
}
#endif
#endif /*__ topology_1_H */


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
