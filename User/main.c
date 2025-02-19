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

//	AddPortButton(MOMENTARY_NO, 2);   //Define a button connected to port P1
//	SetButtonEvents(2, 1, 0, 3, 0, 0, 0, 0, 0,1);    // Activate a click event and a pressed_for_x event for 3 seconds

	StartScastDMAStream(P3, 1, P2, 4, BIDIRECTIONAL, 100, 0xFFFFFFFF, false);
	// put your code here, to run repeatedly.
	while (1) {
//		SampleTempCelsius(&TempBuffer[Index11]);
//		HAL_Delay(50);
//		Index11++;
//		if(Index11 > 100)
//			Index11 =0;
	}
}
void buttonClickedCallback(uint8_t port){

	IND_ON();
//	IND_blink(80);
//	SendMessageToModule(2, CODE_PING, 0);
//
//	SendMessageToModule(3, CODE_PING, 0);

	SendMessageToModule(4, CODE_PING, 0);
	IND_OFF();

//	HAL_Delay(3000);
//
//	SendMessageToModule(BOS_BROADCAST, CODE_PING, 0);
}
/*-----------------------------------------------------------*/
