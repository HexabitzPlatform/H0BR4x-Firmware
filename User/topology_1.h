/*
    BitzOS (BOS) V0.0.0 - Copyright (C) 2016 Hexabitz
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
	 

#define _N	8					// Number of array modules

// Array modules
#define _mod1	1<<3
#define _mod2	2<<3
#define _mod3	3<<3
#define _mod4	4<<3
#define _mod5	5<<3
#define _mod6	6<<3
#define _mod7	7<<3
#define _mod8	8<<3

// Topology
static uint16_t array[_N][7] = {
{ _H01R0, 0, 0, 0, _mod2|P5, 0, _mod3|P4},									// Module 1
{ _H01R0, 0, 0, _mod4|P6, 0, _mod1|P4, 0},									// Module 2
{ _H01R0, 0, 0, 0, _mod1|P6, 0, _mod5|P3},									// Module 3
{ _H01R0, 0, 0, _mod6|P6, _mod7|P6, 0, _mod2|P3},						// Module 4
{ _H01R0, 0, 0, _mod3|P6, 0, _mod7|P4, _mod8|P3},						// Module 5
{ _H01R0, 0, 0, 0, 0, _mod7|P1, _mod4|P3},									// Module 6
{ _H01R0, _mod6|P5, 0, _mod8|P4, _mod5|P5, 0, _mod4|P4},		// Module 7
{ _H01R0, 0, 0, _mod5|P6, _mod7|P3, 0, 0}										// Module 8
};

// Configurations for duplex serial ports
#if ( _module == 1 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 2 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1	
	#define	_P5pol_reversed	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 3 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_reversed	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 4 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_reversed	1
#endif
#if ( _module == 5 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_reversed	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif
#if ( _module == 6 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_reversed	1
#endif
#if ( _module == 7 )
	#define	H01R0	1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_reversed	1
	#define	_P4pol_reversed	1	
	#define	_P5pol_normal	1
	#define	_P6pol_reversed	1
#endif
#if ( _module == 8 )
	#define	H01R0	1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_reversed	1
	#define	_P4pol_normal	1	
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif


#ifdef __cplusplus
}
#endif
#endif /*__ topology_1_H */


/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
