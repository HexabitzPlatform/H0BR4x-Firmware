/**
  ******************************************************************************
  * File Name          : H0BR4_timers.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
		MODIFIED by Hexabitz for BitzOS (BOS) V0.2.4 - Copyright (C) 2017-2021 Hexabitz
    All rights reserved
*/

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/*----------------------------------------------------------------------------*/
/* Configure Timers                                                              */
/*----------------------------------------------------------------------------*/


/* Variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;	/* micro-second delay counter */
TIM_HandleTypeDef htim15;	/* milli-second delay counter */


/*  Micro-seconds timebase init function - TIM14 (16-bit)
*/
void TIM_USEC_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM14_CLK_ENABLE();

	/* Peripheral configuration */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = HAL_RCC_GetPCLK1Freq()/1000000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0xFFFF;
  HAL_TIM_Base_Init(&htim14);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim14, &sMasterConfig);
	
	HAL_TIM_Base_Start(&htim14);
}

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/*  Milli-seconds timebase init function - TIM15 (16-bit)
*/
void TIM_MSEC_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
	
	/* Peripheral clock enable */
	__TIM15_CLK_ENABLE();

	/* Peripheral configuration */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = HAL_RCC_GetPCLK1Freq()/1000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xFFFF;
  HAL_TIM_Base_Init(&htim15);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);
	
	HAL_TIM_Base_Start(&htim15);
}

/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
*/
void StartMicroDelay(uint16_t Delay)
{
	uint32_t t0=0;

	portENTER_CRITICAL();
	
	if (Delay)
	{
		t0 = htim14.Instance->CNT;

		while(htim14.Instance->CNT - t0 <= Delay) {};
	}
	
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
*/
void StartMilliDelay(uint16_t Delay)
{
	uint32_t t0=0;
	
	portENTER_CRITICAL();
	
	if (Delay)
	{
		t0 = htim15.Instance->CNT;

		while(htim15.Instance->CNT - t0 <= Delay) {};
	}
	
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
