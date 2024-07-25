/*
 BitzOS (BOS) V0.3.5 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/
float x11, y11 , z11;
float x22, y22 , z22;
float temp1 , temp2;

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

//		SampleGyroDPS(&x11 , &y11, &z11);
//
//		SampleAccG(&x22, &y22, &z22);
//
//		SampleTempCelsius(&temp1);
//		SampleTempFahrenheit(&temp2);

	}
}

/*-----------------------------------------------------------*/
