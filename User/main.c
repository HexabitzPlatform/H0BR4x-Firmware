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

		messageParams[0] = 20;
		SendMessageToModule(1, CODE_H01R0_ON, 1);
			Delay_ms(500);
			messageParams[0] = 100;
			SendMessageToModule(1, CODE_H01R0_ON, 1);



	}
}

/*-----------------------------------------------------------*/
