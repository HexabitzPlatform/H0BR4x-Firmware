/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved
 
 File Name     : H0BR4.h
 Description   : Header file for module H0BR4.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H0BR4_H
#define H0BR4_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H0BR4_MemoryMap.h"
#include "H0BR4_uart.h"
#include "H0BR4_i2c.h"
#include "H0BR4_gpio.h"
#include "H0BR4_dma.h"
#include "H0BR4_inputs.h"
#include "H0BR4_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H0BR4


/* Port-related definitions */
#define	NumOfPorts			6

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 
#define _P6

/* Define available USARTs */
#define _Usart1 1
#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */

#define P1uart &huart4
#define P2uart &huart2
#define P3uart &huart3
#define P4uart &huart1
#define P5uart &huart5
#define P6uart &huart6


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

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
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

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6

/* Module-specific Definitions */
#define IMU_INT1_PORT									GPIOB
#define IMU_INT1_PIN									GPIO_PIN_6
#define IMU_INT1_GPIO_CLK()						__GPIOB_CLK_ENABLE();
#define IMU_INT2_PORT									GPIOB
#define IMU_INT2_PIN									GPIO_PIN_5
#define IMU_INT2_GPIO_CLK()						__GPIOB_CLK_ENABLE();
#define MAG_INT_PORT									GPIOA
#define MAG_INT_PIN										GPIO_PIN_15
#define MAG_INT_GPIO_CLK()						__GPIOA_CLK_ENABLE();
#define XL_INT1_PORT									GPIOD
#define XL_INT1_PIN										GPIO_PIN_1
#define XL_INT1_GPIO_CLK()						__GPIOD_CLK_ENABLE();
#define XL_INT2_PORT									GPIOD
#define XL_INT2_PIN										GPIO_PIN_0
#define XL_INT2_GPIO_CLK()						__GPIOD_CLK_ENABLE();

#define _MEMS_I2C2_SDA_PORT       		GPIOB
#define _MEMS_I2C2_SDA_PIN            GPIO_PIN_4
#define _MEMS_I2C2_SDA_GPIO_CLK()     __GPIOB_CLK_ENABLE();
#define _MEMS_I2C2_SCL_PORT           GPIOB
#define _MEMS_I2C2_SCL_PIN            GPIO_PIN_3
#define _MEMS_I2C2_SCL_GPIO_CLK()     __GPIOB_CLK_ENABLE();

#define NUM_MODULE_PARAMS		13

/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* Module_Status Type Definition */
typedef enum {
	H0BR4_OK =0, H0BR4_ERR_UnknownMessage, H0BR4_ERR_GYRO, H0BR4_ERR_ACC, H0BR4_ERR_MAG, H0BR4_ERR_LSM6DS3, H0BR4_ERR_LSM303, H0BR4_ERR_BUSY, H0BR4_ERR_TIMEOUT, H0BR4_ERR_IO, H0BR4_ERR_TERMINATED, H0BR4_ERR_WrongParams, H0BR4_ERROR =25
} Module_Status;

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_14

/* Export UART variables */
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

/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
Module_Status SampleGyroMDPS(int *gyroX,int *gyroY,int *gyroZ);
Module_Status SampleGyroRaw(int16_t *gyroX,int16_t *gyroY,int16_t *gyroZ);

Module_Status SampleGyroDPS(float *x,float *y,float *z);
Module_Status SampleGyroDPSToBuf(float *buffer);
Module_Status SampleGyroDPSToString(char *cstring,size_t maxLen);
Module_Status SampleGyroDPSToPort(uint8_t port,uint8_t module);

Module_Status SampleAccMG(int *accX,int *accY,int *accZ);
Module_Status SampleAccRaw(int16_t *accX,int16_t *accY,int16_t *accZ);

Module_Status SampleAccG(float *x,float *y,float *z);
Module_Status SampleAccGToBuf(float *buffer);
Module_Status SampleAccGToString(char *cstring,size_t maxLen);
Module_Status SampleAccGToPort(uint8_t port,uint8_t module);

Module_Status SampleMagMGauss(int *magX,int *magY,int *magZ);
Module_Status SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ);

Module_Status SampleMagMGaussToBuf(float *buffer);
Module_Status SampleMagMGaussToString(char *cstring,size_t maxLen);
Module_Status SampleMagMGaussToPort(uint8_t port,uint8_t module);

Module_Status SampleTempCelsius(float *temp);
Module_Status SampleTempFahrenheit(float *temp);

Module_Status SampleTempCToPort(uint8_t port,uint8_t module);
Module_Status SampleTempCToString(char *cstring,size_t maxLen);

Module_Status StreamGyroDPSToPort(uint8_t port,uint8_t module,uint32_t period,uint32_t timeout);
Module_Status StreamGyroDPSToCLI(uint32_t period,uint32_t timeout);
Module_Status StreamGyroDPSToBuffer(float *buffer,uint32_t period,uint32_t timeout);

Module_Status StreamAccGToPort(uint8_t port,uint8_t module,uint32_t period,uint32_t timeout);
Module_Status StreamAccGToCLI(uint32_t period,uint32_t timeout);
Module_Status StreamAccGToBuffer(float *buffer,uint32_t period,uint32_t timeout);

Module_Status StreamMagMGaussToPort(uint8_t port,uint8_t module,uint32_t period,uint32_t timeout);
Module_Status StreamMagMGaussToCLI(uint32_t period,uint32_t timeout);
Module_Status StreamMagMGaussToBuffer(float *buffer,uint32_t period,uint32_t timeout);

Module_Status StreamTempCToPort(uint8_t port,uint8_t module,uint32_t period,uint32_t timeout);
Module_Status StreamTempCToCLI(uint32_t period,uint32_t timeout);
Module_Status StreamTempCToBuffer(float *buffer,uint32_t period,uint32_t timeout);

void stopStreamMems(void);

void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */


#endif /* H0BR4_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
