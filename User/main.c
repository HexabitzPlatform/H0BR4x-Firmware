/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/
uint8_t TempBuffer[100];
uint16_t Index11;

uint8_t var = 0;
VariableFormat_t format;
/* Private Function Prototypes *********************************************/


/* Main Function ***********************************************************/
int main(void) {

	/* Initialize Module &  BitzOS */
	Module_Init();

	/* Don't place your code here */
	for (;;) {
	}
}

/***************************************************************************/
/* User Task */
void UserTask(void *argument) {

//	for (uint8_t i = 1 ; i<= 100 ; i++) {
//		TempBuffer[i] = i;
//	}
//StreamPortToPort(P3, 1, P3, 4, BIDIRECTIONAL, 100, 0xffffff, 0); // ok
//StreamPortToMemory(P3, 4, 100, 0xffffff, 0);
//StreamMemoryToPort(3, 4, (uint8_t *)TempBuffer, 100, 0xffffff, 0); // ok
//StreamMemoryToMemory(4, uint8_t *pBuffer, 100, 0xffffff, 0);

//	SwapUartPins(GetUart(P1),REVERSED);
//	SwapUartPins(GetUart(P2),REVERSED);
//	SwapUartPins(GetUart(P3),REVERSED);
//	SwapUartPins(GetUart(P4),REVERSED);
//	SwapUartPins(GetUart(P5),REVERSED);
//	SwapUartPins(GetUart(P6),REVERSED);

//	AddButton(P3, MOMENTARY_NO, CLICKED);

	// put your code here, to run repeatedly.
	while (1) {

/* Problem 1: */
///* when using 0xffffff for data count the module is being reset */
//		for (uint8_t i = 1 ; i<= 100 ; i++) {
//			TempBuffer[i] = i;
//
//			StreamMemoryToPort(3, 4, TempBuffer, 100 , 0xffffff, 0);
//			HAL_Delay(10);
//
//		}

	}
}

//void buttonClickedCallback(uint8_t port){
//	uint8_t A =0;
//	A++;
//}

//void buttonDblClickedCallback(uint8_t port) {
//	uint8_t A =0;
//	A++;
//}
/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
