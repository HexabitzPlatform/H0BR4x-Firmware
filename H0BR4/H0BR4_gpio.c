/**
  ******************************************************************************
  * File Name          : H0BR4_gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/*
		MODIFIED by Hexabitz for BitzOS (BOS) V0.2.2 - Copyright (C) 2017-2020 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/

/** Pinout Configuration
*/
void GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();		// for HSE and Boot0
	
	IND_LED_Init();
	MEMS_GPIO_Init();
}

//-- Configure indicator LED
void IND_LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.Pin = _IND_LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(_IND_LED_PORT, &GPIO_InitStruct);
}

void MEMS_GPIO_Init(void)
{
// TODO: Enable INT pins and their interrupts.
  GPIO_InitTypeDef GPIO_InitStruct;

  /**I2C2 GPIO Configuration
  PB13     ------> I2C2_SCL
  PB14     ------> I2C2_SDA
  */
	
  GPIO_InitStruct.Pin = _MEMS_I2C2_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_I2C2;
  HAL_GPIO_Init(_MEMS_I2C2_SCL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = _MEMS_I2C2_SDA_PIN;
  HAL_GPIO_Init(_MEMS_I2C2_SDA_PORT, &GPIO_InitStruct);

  /* Peripheral clock enable */
  __HAL_RCC_I2C2_CLK_ENABLE();
	
  /* I2C2 interrupt Init */
  HAL_NVIC_SetPriority(I2C2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C2_IRQn);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
