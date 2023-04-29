/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : topology.h
 Description   : Array topology definition.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __topology_H
#define __topology_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#define __N	7					// Number of array modules

// Array modules
#define _mod1	1<<3
#define _mod2	2<<3
#define _mod3	3<<3
#define _mod4	4<<3
#define _mod5	5<<3
#define _mod6	6<<3
#define _mod7	7<<3
// Topology
static uint16_t array[__N ][7] ={
	{_H01R0, _mod2 | P1, _mod3 | P1, _mod4 | P1, _mod5 | P1, _mod6 | P1, _mod7 | P1}, 								 // Module 1

	{_H0BR4, _mod1 | P1,0, 0,0, 0,0},					    	     // Module 2
	{_H23R0, _mod1 | P2,0, 0,0, 0,0},					    	     // Module 3
	{_H07R3, _mod1 | P3,0, 0,0, 0,0},					    	     // Module 4
	{_H0AR9, _mod1 | P4,0, 0,0, 0,0},					    	     // Module 5
	{_H0FR6, _mod1 | P5,0, 0,0, 0,0},					    	     // Module 6
	{_H0FR7, _mod1 | P6,0, 0,0, 0,0},					    	     // Module 7
};

// Configurations for duplex serial ports
#if ( _module == 1 )
	#define	H01R0	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 2 )
	#define	H0BR4			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1

#endif

#if ( _module == 3 )
	#define	H23R0	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 4 )
	#define	 H07R3	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 5 )
	#define	 H0AR9	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 6 )
	#define	H0FR6	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 7 )
	#define	 H0FR7	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif


#ifdef __cplusplus
}
#endif
#endif /*__ topology_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
