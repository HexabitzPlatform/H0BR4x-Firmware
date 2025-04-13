/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name          : H0BR4_i2c.c
 Description        : This file provides code for the configuration
 of the I2C instances.

 */

/* Includes ****************************************************************/
#include "BOS.h"
#include <string.h>
#include <stdio.h>

/* Exported Variables ******************************************************/
I2C_HandleTypeDef hi2c2;

/* Exported Functions ******************************************************/
void MX_I2C_Init(void);
void MX_I2C2_Init(void);

/***************************************************************************/
/* Configure I2C ***********************************************************/
/***************************************************************************/

/** I2C Configuration
 */
void MX_I2C_Init(void){
	/* GPIO Ports Clock Enable */
	__GPIOC_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();

	MX_I2C2_Init();

}

/***************************************************************************/
/* I2C2 init function */
void MX_I2C2_Init(void){

	/* Initialize I2C2 peripheral */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing =0x00C12166; // Fast mode (400 kHz)
	hi2c2.Init.OwnAddress1 =0; // No specific address required for master mode
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT; // 7-bit addressing mode
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // Disable dual address mode
	hi2c2.Init.OwnAddress2 =0; // Not used, set to 0
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK; // No mask for second address
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // Disable general call
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE; // Disable clock stretching
	HAL_I2C_Init(&hi2c2);

	/** Configure Analogue filter */
	HAL_I2CEx_ConfigAnalogFilter(&hi2c2,I2C_ANALOGFILTER_ENABLE); // Enable analog filter

	/** Configure Digital filter */
	HAL_I2CEx_ConfigDigitalFilter(&hi2c2,0); // Digital filter set to 0 (disabled)
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
