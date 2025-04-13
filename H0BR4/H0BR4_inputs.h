/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H0BR4_inputs.h
 Description   : header file for Bitz digital and analog inputs.
 
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_adc_ex.h"
#include "string.h"

/***************************************************************************/
/* Exported Functions Prototypes *******************************************/
/***************************************************************************/
extern void ReadTempAndVref(float *temp,float *Vref);
extern void ReadADCChannel(uint8_t Port,char *side,float *ADC_Value);
extern void ADCSelectChannel(uint8_t ADC_port,char *side);
extern void Deinit_ADC_Channel(uint8_t port);
extern float GetReadPrecentage(uint8_t port,float *precentageValue);

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
