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
static void LSM6D3Init(void);
static void LSM303Init(void);


/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE LSM6DS3GetGyroCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

const CLI_Command_Definition_t LSM6DS3GyroCommandDefinition = {
	(const int8_t *) "gyro",
	(const int8_t *) "Command to get Gyro.\r\n",
	LSM6DS3GetGyroCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3AccCommandDefinition = {
	(const int8_t *) "acc",
	(const int8_t *) "Command to get Accelerometer.\r\n",
	LSM6DS3GetAccCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3TempCommandDefinition = {
	(const int8_t *) "temperature",
	(const int8_t *) "Command to get Temperature.\r\n",
	LSM6DS3GetTempCommand,
	0
};



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
	
	// TODO: Initialize I2C
	
	// TODO: Initialize Sensors
	LSM6D3Init();
	LSM303Init();

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
	FreeRTOS_CLIRegisterCommand(&LSM6DS3GyroCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&LSM6DS3AccCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&LSM6DS3TempCommandDefinition);
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

static void LSM6D3Init(void)
{
	
}

static void LSM303Init(void)
{
	
}

/* -----------------------------------------------------------------------
	|																APIs	 																 	|
   ----------------------------------------------------------------------- 
*/




/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

static portBASE_TYPE LSM6DS3GetGyroCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something 
	*pcWriteBuffer = '\0';
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something 
	*pcWriteBuffer = '\0';
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something 
	*pcWriteBuffer = '\0';
	return pdFALSE;
}



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
