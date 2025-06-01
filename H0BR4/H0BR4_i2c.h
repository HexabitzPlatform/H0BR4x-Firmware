/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H0BR4_i2c.h
 Description: Declares functions and variables for initializing I2C2 peripheral,
 supporting fast mode (400 kHz) communication with GPIO (SCL, SDA) setup.
 Enabled Peripherals: I2C2, GPIO (Port A).
 */
/* Define to prevent recursive inclusion ***********************************/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "stm32g0xx_hal.h"

/* Exported Variables ******************************************************/
extern I2C_HandleTypeDef hi2c2;

/* Exported Functions ******************************************************/

extern void MX_I2C2_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__i2c_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
