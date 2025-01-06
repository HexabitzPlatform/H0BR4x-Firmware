/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
float TempBuffer[100];
uint16_t Index11;
/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void) {

	Module_Init();		//Initialize Module &  BitzOS

	//Don't place your code here.
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument) {


	// put your code here, to run repeatedly.
	while (1) {
		SampleTempCelsius(&TempBuffer[Index11]);
		HAL_Delay(50);
		Index11++;
		if(Index11 > 100)
			Index11 =0;
	}
}

/*-----------------------------------------------------------*/
