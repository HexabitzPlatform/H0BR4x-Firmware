/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved
 
 File Name     : H0BR4.h
 Description   : Header file for module H0BR4.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H0BR4_H
#define H0BR4_H

/* Includes ****************************************************************/
#include "BOS.h"
#include "H0BR4_MemoryMap.h"
#include "H0BR4_uart.h"
#include "H0BR4_i2c.h"
#include "H0BR4_gpio.h"
#include "H0BR4_dma.h"
#include "H0BR4_inputs.h"
#include "H0BR4_eeprom.h"

/* Exported Macros *********************************************************/
#define	modulePN		_H0BR4

/* Port-related Definitions */
#define	NumOfPorts		6
#define P_PROG 			P2		/* ST factory bootloader UART */

/* Define Available Ports */
#define _P1
#define _P2
#define _P3
#define _P4
#define _P5
#define _P6

/* Define Available USARTs */
#define _Usart1
#define _Usart2
#define _Usart3
#define _Usart4
#define _Usart5
#define _Usart6

/* Port-UART Mapping */
#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart6
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart3

/* Module-specific Hardware Definitions ************************************/
/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_9
#define	USART1_RX_PIN		GPIO_PIN_10
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_8
#define	USART3_RX_PIN		GPIO_PIN_9
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_4
#define	USART6_RX_PIN		GPIO_PIN_5
#define	USART6_TX_PORT		GPIOA
#define	USART6_RX_PORT		GPIOA
#define	USART6_AF			GPIO_AF8_USART6

/* Module-specific Hardware Definitions */
#define _MEMS_I2C2_SDA_PORT       		  GPIOA
#define _MEMS_I2C2_SDA_PIN                GPIO_PIN_6
#define _MEMS_I2C2_SDA_GPIO_CLK()         __GPIOB_CLK_ENABLE();
#define _MEMS_I2C2_SCL_PORT               GPIOA
#define _MEMS_I2C2_SCL_PIN                GPIO_PIN_7
#define _MEMS_I2C2_SCL_GPIO_CLK()         __GPIOB_CLK_ENABLE();

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_12

/* Module-specific Macro Definitions ***************************************/
#define MOVEMENT_DETECTED       1    /* Macro for movement detected */
#define INITIAL_IDLE_STATE      2    /* Macro for initial idle state */
#define SENSITIVITY             0x03 /* Define the threshold for wake-up sensitivity */

#define MIN_MEMS_PERIOD_MS		100
#define MAX_MEMS_TIMEOUT_MS		0xFFFFFFFF
#define NUM_MODULE_PARAMS		13

/* Macros definitions */
#define STREAM_MODE_TO_PORT      1
#define STREAM_MODE_TO_TERMINAL  2
/* Module-specific Type Definition *****************************************/
/* Module-status Type Definition */
typedef enum {
	H0BR4_OK =0,
	H0BR4_ERR_UnknownMessage,
	H0BR4_ERR_TERMINATED,
	H0BR4_ERR_WrongParams,
	H0BR4_ERROR =25
} Module_Status;

/* IMU Signal type */
typedef enum {
	ACC =0,
	GYRO,
	MAG,
	TEMP
} All_Data;

/* Exported UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
Module_Status SampleAccG(float *accX,float *accY,float *accZ);
Module_Status SampleGyroDPS(float *gyroX,float *gyroY,float *gyroZ);
Module_Status SampleMagMGauss(int *magX,int *magY,int *magZ);
Module_Status SampleTempCelsius(float *temp);
Module_Status SampleTempFahrenheit(float *temp);

Module_Status SampleGyroRaw(int16_t *gyroX,int16_t *gyroY,int16_t *gyroZ);
Module_Status SampleAccRaw(int16_t *accX,int16_t *accY,int16_t *accZ);
Module_Status SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ);

Module_Status SampleToPort(uint8_t dstModule, uint8_t dstPort, All_Data dataFunction);
Module_Status StreamtoPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout);
Module_Status StreamToBuffer(float *buffer,All_Data function, uint32_t Numofsamples, uint32_t timeout);

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

#endif /* H0BR4_H */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
