/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
uint8_t TempBuffer[100];
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

//	for (uint8_t i = 1 ; i<= 100 ; i++) {
//		TempBuffer[i] = i;
//	}
//StreamPortToPort(P3, 1, P3, 4, BIDIRECTIONAL, 100, 0xffffff, 0); // ok
//StreamPortToMemory(P3, 4, 100, 0xffffff, 0);
//StreamMemoryToPort(3, 4, (uint8_t *)TempBuffer, 100, 0xffffff, 0); // ok
//StreamMemoryToMemory(4, uint8_t *pBuffer, 100, 0xffffff, 0);



	// put your code here, to run repeatedly.
	while (1) {
/* Problem 1: */
/* when using 0xffffff for data count the module is being reset */
//		for (uint8_t i = 1 ; i<= 100 ; i++) {
//			TempBuffer[i] = i;
//
//			StreamMemoryToPort(3, 4, TempBuffer, 100 , 0xffffff, 0);
//			HAL_Delay(10);
//
//		}



//		SampleTempCelsius(&TempBuffer[Index11]);
//		HAL_Delay(50);
//		Index11++;
//		if(Index11 > 100)
//			Index11 =0;
	}
}

/*-----------------------------------------------------------*/
