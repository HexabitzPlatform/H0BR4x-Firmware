/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();		//Initialize Module &  BitzOS



	//Don't place your code here.
	for(;;){}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){



	// put your code here, to run repeatedly.
	while(1){

			SendMessageToModule(1, CODE_PING, 0);
//			//Delay_ms(500);
//			SendMessageToModule(3, CODE_PING, 0);
//			//Delay_ms(500);
//			SendMessageToModule(4, CODE_PING, 0);
//			//Delay_ms(500);
//			SendMessageToModule(5, CODE_PING, 0);
//			//Delay_ms(500);
//			SendMessageToModule(6, CODE_PING, 0);
//			//Delay_ms(500);
//			SendMessageToModule(7, CODE_PING, 0);
//			//Delay_ms(500);

	}
}

/*-----------------------------------------------------------*/
