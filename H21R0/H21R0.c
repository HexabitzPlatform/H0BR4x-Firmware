/*
    BitzOS (BOS) V0.1.4 - Copyright (C) 2017 Hexabitz
    All rights reserved

    File Name     : H0BR4.c
    Description   : Source code for module H0BR4.
										IMU (ST LSM6DS3TR) + Digital Compass (ST LSM303AGRTR)
		
		Required MCU resources : 
		
			>> USARTs 1,2,3,4,5,6 for module ports.
			>> I2C2 for LSM6DS3TR and LSM303AGRTR communication.
			>> GPIOB 12, GPIOA 6 for LSM6DS3TR IMU_INT1 and IMU_INT2.
			>> GPIOB 1, GPIOB 0 for LSM303AGRTR XL_INT1 and XL_INT2.
			>> GPIOA 7 for LSM303AGRTR MAG_INT.
			
*/
	
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/	


/* Create CLI commands --------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|												 Private Functions	 														|
   ----------------------------------------------------------------------- 
*/

/* --- H0BR4 module initialization. 
*/
void Module_Init(void)
{
	/* Peripheral clock enable */

	/* Array ports */
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_USART5_UART_Init();
  MX_USART6_UART_Init();
	

}

/*-----------------------------------------------------------*/

/* --- H0BR4 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H0BR4_OK;
	
	switch (code)
	{

		
		default:
			result = H0BR4_ERR_UnknownMessage;
			break;
	}			

	return result;	
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands 
*/
void RegisterModuleCLICommands(void)
{

}

/*-----------------------------------------------------------*/

/* --- Get the port for a given UART. 
*/
uint8_t GetPort(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART4)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART6)
		return P3;
	else if (huart->Instance == USART3)
		return P4;
	else if (huart->Instance == USART1)
		return P5;
	else if (huart->Instance == USART5)
		return P6;

	return 0;
}

/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/



/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
