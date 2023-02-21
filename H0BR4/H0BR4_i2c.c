/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name          : H0AR9_i2c.c
 Description        : This file provides code for the configuration
 of the I2C instances.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <string.h>
#include <stdio.h>
#include "LSM6DS3.h"
#include "LSM303AGR_ACC.h"
#include "LSM303AGR_MAG.h"


//initialize i2c
 void MX_I2C_Init(void);
 void MX_I2C2_Init(void);


I2C_HandleTypeDef hi2c2;




/*----------------------------------------------------------------------------*/
/* Configure I2C                                                             */
/*----------------------------------------------------------------------------*/

/** I2C Configuration
*/
 void MX_I2C_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();   // for HSE and Boot0
	 // __HAL_RCC_GPIOF_CLK_ENABLE();
	 	//  __HAL_RCC_GPIOA_CLK_ENABLE();
  MX_I2C2_Init();

}



 void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);


  /** Configure Analogue filter
  */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);


  /** Configure Digital filter
  */
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);

  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

 uint8_t LSM6DS3_I2C_Write(void *handle,uint8_t WriteAddr,uint8_t *pBuffer,uint16_t nBytesToWrite){
 	if(HAL_I2C_Mem_Write(handle,LSM6DS3_ACC_GYRO_I2C_ADDRESS_HIGH,WriteAddr,sizeof(WriteAddr),pBuffer,nBytesToWrite,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }

 uint8_t LSM6DS3_I2C_Read(void *handle,uint8_t ReadAddr,uint8_t *pBuffer,uint16_t nBytesToRead){
 	if(HAL_I2C_Mem_Read(handle,LSM6DS3_ACC_GYRO_I2C_ADDRESS_HIGH,ReadAddr,sizeof(ReadAddr),pBuffer,nBytesToRead,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }

 uint8_t LSM303AGR_ACC_I2C_Write(void *handle,uint8_t WriteAddr,uint8_t *pBuffer,uint16_t nBytesToWrite){
 	if(HAL_I2C_Mem_Write(handle,LSM303AGR_ACC_I2C_ADDRESS,WriteAddr,sizeof(WriteAddr),pBuffer,nBytesToWrite,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }

 uint8_t LSM303AGR_ACC_I2C_Read(void *handle,uint8_t ReadAddr,uint8_t *pBuffer,uint16_t nBytesToRead){
 	if(HAL_I2C_Mem_Read(handle,LSM303AGR_ACC_I2C_ADDRESS,ReadAddr,sizeof(ReadAddr),pBuffer,nBytesToRead,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }

 uint8_t LSM303AGR_MAG_I2C_Write(void *handle,uint8_t WriteAddr,uint8_t *pBuffer,uint16_t nBytesToWrite){
 	if(HAL_I2C_Mem_Write(handle,LSM303AGR_MAG_I2C_ADDRESS,WriteAddr,sizeof(WriteAddr),pBuffer,nBytesToWrite,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }

 uint8_t LSM303AGR_MAG_I2C_Read(void *handle,uint8_t ReadAddr,uint8_t *pBuffer,uint16_t nBytesToRead){
 	if(HAL_I2C_Mem_Read(handle,LSM303AGR_MAG_I2C_ADDRESS,ReadAddr,sizeof(ReadAddr),pBuffer,nBytesToRead,100) != HAL_OK){
 		return 1;
 	}
 	return 0;
 }
/*-----------------------------------------------------------*/

