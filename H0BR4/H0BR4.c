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

#include "H0BR4_LSM6DS3_driver.h"
#include "H0BR4_LSM303AGR_ACC_driver.h"
#include "H0BR4_LSM303AGR_MAG_driver.h"

#include <math.h>

#define LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G  1.5  /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */


/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static Module_Status LSM6D3Init(void);
static Module_Status LSM303AccInit(void);
static Module_Status LSM303MagInit(void);

static Module_Status LSM6D3GetGyro(int *gyroX, int *gyroY, int *gyroZ);
static Module_Status LSM6D3GetGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

static Module_Status LSM6D3GetAcc(int *accX, int *accY, int *accZ);
static Module_Status LSM6D3GetAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ);

static Module_Status LSM303MagGetAxes(int *magX, int *magY, int *magZ);
static Module_Status LSM303MagGetRawAxes(int16_t *magX, int16_t *magY, int16_t *magZ);


/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE LSM6DS3GetGyroCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM303GetMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);


const CLI_Command_Definition_t LSM6DS3GyroCommandDefinition = {
	(const int8_t *) "gyro",
	(const int8_t *) "Command to get Filtered and calibrated Gyro X,Y,Z values in dps.\r\n",
	LSM6DS3GetGyroCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3AccCommandDefinition = {
	(const int8_t *) "acc",
	(const int8_t *) "Command to get Filtered and calibrated Accelerometer X,Y,Z values in g.\r\n",
	LSM6DS3GetAccCommand,
	0
};

const CLI_Command_Definition_t LSM303MagCommandDefinition = {
	(const int8_t *) "mag",
	(const int8_t *) "Command to get Filtered and calibrated Magnetic strength X,Y,Z values.\r\n",
	LSM303GetMagCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3TempCommandDefinition = {
	(const int8_t *) "temperature",
	(const int8_t *) "Command to get Filtered and calibrated Temperature values in Celsius.\r\n",
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
	MX_I2C_Init();
	
	// TODO: Initialize Sensors
	LSM6D3Init();
	//LSM303AccInit();
	//LSM303MagInit();

}

/*-----------------------------------------------------------*/

/* --- H0BR4 message processing task. 
*/
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst)
{
	Module_Status result = H0BR4_OK;
	
	switch (code)
	{
		case CODE_H0BR4_GET_GYRO:
		{
			int gyroX = 0, gyroY = 0, gyroZ = 0;
			if (LSM6D3GetGyro(&gyroX, &gyroY, &gyroZ) != H0BR4_OK)
				return H0BR4_ERR_LSM6DS3;
			
			messageParams[0] = lowByte(gyroX); messageParams[1] = highByte(gyroX);
			messageParams[2] = lowByte(gyroY); messageParams[3] = highByte(gyroY);
			messageParams[4] = lowByte(gyroZ); messageParams[5] = highByte(gyroZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_GYRO, 6);
			break;
		}
		case CODE_H0BR4_GET_RAW_GYRO:
		{
			int16_t gyroX = 0, gyroY = 0, gyroZ = 0;
			if (LSM6D3GetGyroRaw(&gyroX, &gyroY, &gyroZ) != H0BR4_OK)
				return H0BR4_ERR_LSM6DS3;
			
			messageParams[0] = lowByte(gyroX); messageParams[1] = highByte(gyroX);
			messageParams[2] = lowByte(gyroY); messageParams[3] = highByte(gyroY);
			messageParams[4] = lowByte(gyroZ); messageParams[5] = highByte(gyroZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_RAW_GYRO, 6);
			break;
		}
		case CODE_H0BR4_GET_ACC:
		{
			int accX = 0, accY = 0, accZ = 0;
			if (LSM6D3GetAcc(&accX, &accY, &accZ) != H0BR4_OK)
				return H0BR4_ERR_LSM6DS3;
			
			messageParams[0] = lowByte(accX); messageParams[1] = highByte(accX);
			messageParams[2] = lowByte(accY); messageParams[3] = highByte(accY);
			messageParams[4] = lowByte(accZ); messageParams[5] = highByte(accZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_ACC, 6);
			break;
		}
		case CODE_H0BR4_GET_RAW_ACC:
		{
			int16_t accX = 0, accY = 0, accZ = 0;
			if (LSM6D3GetAccRaw(&accX, &accY, &accZ) != H0BR4_OK)
				return H0BR4_ERR_LSM6DS3;
			
			messageParams[0] = lowByte(accX); messageParams[1] = highByte(accX);
			messageParams[2] = lowByte(accY); messageParams[3] = highByte(accY);
			messageParams[4] = lowByte(accZ); messageParams[5] = highByte(accZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_RAW_ACC, 6);
			break;
		}
		case CODE_H0BR4_GET_MAG:
		{
			int magX = 0, magY = 0, magZ = 0;
			if (LSM303MagGetAxes(&magX, &magY, &magZ) != H0BR4_OK)
				return H0BR4_ERR_LSM303;
			
			messageParams[0] = lowByte(magX); messageParams[1] = highByte(magX);
			messageParams[2] = lowByte(magY); messageParams[3] = highByte(magY);
			messageParams[4] = lowByte(magZ); messageParams[5] = highByte(magZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_MAG, 6);
			break;
		}
		case CODE_H0BR4_GET_RAW_MAG:
		{
			int16_t magX = 0, magY = 0, magZ = 0;
			if (LSM303MagGetRawAxes(&magX, &magY, &magZ) != H0BR4_OK)
				return H0BR4_ERR_LSM303;
			
			messageParams[0] = lowByte(magX); messageParams[1] = highByte(magX);
			messageParams[2] = lowByte(magY); messageParams[3] = highByte(magY);
			messageParams[4] = lowByte(magZ); messageParams[5] = highByte(magZ);
			SendMessageFromPort(port, myID, dst, CODE_H0BR4_RESULT_RAW_MAG, 6);
			break;
		}
		case CODE_H0BR4_GET_TEMP:
		{
			break;
		}
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
	FreeRTOS_CLIRegisterCommand(&LSM303MagCommandDefinition);
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

static Module_Status LSM6D3Enable(void)
{
	// Enable register address automatically incremented during a multiple byte access with a serial interface 
	if (LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(&hi2c2, LSM6DS3_ACC_GYRO_IF_INC_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	// Bypass Mode
	if (LSM6DS3_ACC_GYRO_W_FIFO_MODE(&hi2c2, LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS))
		return H0BR4_ERR_LSM6DS3;
	
	// TODO: Set ODR to 0x00 (NA)
	
	return H0BR4_OK;
}

static Module_Status LSM6D3SetupGyro(void)
{
	// Gyroscope ODR Init
	if (LSM6DS3_ACC_GYRO_W_ODR_G(&hi2c2, LSM6DS3_ACC_GYRO_ODR_G_13Hz))
		return H0BR4_ERR_LSM303;
	
	// Gyroscope FS Init
	if (LSM6DS3_ACC_GYRO_W_FS_G(&hi2c2, LSM6DS3_ACC_GYRO_FS_G_2000dps))
		return H0BR4_ERR_LSM6DS3;
	
	// Gyroscope Axes Status Init
	if (LSM6DS3_ACC_GYRO_W_XEN_G(&hi2c2, LSM6DS3_ACC_GYRO_XEN_G_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_YEN_G(&hi2c2, LSM6DS3_ACC_GYRO_YEN_G_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_ZEN_G(&hi2c2, LSM6DS3_ACC_GYRO_ZEN_G_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	return H0BR4_OK;
}

static Module_Status LSM6D3SetupAcc(void)
{
	// Accelerometer ODR Init
	if (LSM6DS3_ACC_GYRO_W_ODR_XL(&hi2c2, LSM6DS3_ACC_GYRO_ODR_XL_13Hz))
		return H0BR4_ERR_LSM6DS3;
	
	// Accelerometer FS Init
	if (LSM6DS3_ACC_GYRO_W_FS_XL(&hi2c2, LSM6DS3_ACC_GYRO_FS_XL_2g))
		return H0BR4_ERR_LSM6DS3;
	
	// Accelerometer Axes Status Init
	if (LSM6DS3_ACC_GYRO_W_XEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_XEN_XL_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_YEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_YEN_XL_ENABLED))
		return H0BR4_ERR_LSM6DS3;
		
	if (LSM6DS3_ACC_GYRO_W_ZEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED))
		return H0BR4_ERR_LSM6DS3;
	
	return H0BR4_OK;
}



static Module_Status LSM6D3GetGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
	uint8_t temp[6];
	if (LSM6DS3_ACC_GYRO_GetRawGyroData(&hi2c2, temp))
		return H0BR4_ERR_LSM6DS3;
	
	*gyroX = concatBytes(temp[1], temp[0]);
	*gyroY = concatBytes(temp[3], temp[2]);
	*gyroZ = concatBytes(temp[5], temp[4]);
	
	return H0BR4_OK;
}

static Module_Status LSM6D3GetGyro(int *gyroX, int *gyroY, int *gyroZ)
{
	int buff[3];
	if (LSM6DS3_ACC_Get_AngularRate(&hi2c2, buff, 0))
		return H0BR4_ERR_LSM6DS3;
	
	*gyroX = buff[0];
	*gyroY = buff[1];
	*gyroZ = buff[2];
	
	return H0BR4_OK;
}

static Module_Status LSM6D3GetAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ)
{
	uint8_t temp[6];
	if (LSM6DS3_ACC_GYRO_GetRawAccData(&hi2c2, temp))
		return H0BR4_ERR_LSM6DS3;
	
	*accX = concatBytes(temp[1], temp[0]);
	*accY = concatBytes(temp[3], temp[2]);
	*accZ = concatBytes(temp[5], temp[4]);
	
	return H0BR4_OK;
}

static Module_Status LSM6D3GetAcc(int *accX, int *accY, int *accZ)
{
	int buff[3];
	if (LSM6DS3_ACC_Get_Acceleration(&hi2c2, buff, 0))
		return H0BR4_ERR_LSM6DS3;
	
	*accX = buff[0];
	*accY = buff[1];
	*accZ = buff[2];
	
	return H0BR4_OK;
}

static Module_Status LSM6D3Init(void)
{
	// Common Init
	Module_Status status = H0BR4_OK;
	if ((status = LSM6D3Enable()) != H0BR4_OK)
		return status;
	
	if ((status = LSM6D3SetupGyro()) != H0BR4_OK)
		return status;
	
	if ((status = LSM6D3SetupAcc()) != H0BR4_OK)
		return status;
	
	// Configure Interrupt Lines
	
	return status;
}


static Module_Status LSM303AccInit(void)
{
	Module_Status status = H0BR4_OK;
	
	// Check WhoAmI
	uint8_t who_am_i;
	if ( LSM303AGR_ACC_R_WHO_AM_I(&hi2c2, &who_am_i ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	if (who_am_i != LSM303AGR_ACC_WHO_AM_I)
		return H0BR4_ERR_LSM303;
	
	// Do Self test
	
	
	
	// Enable Block Data update
	if (LSM303AGR_ACC_W_BlockDataUpdate(&hi2c2, LSM303AGR_ACC_BDU_ENABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// FIFO Mode: ByPass
	if (LSM303AGR_ACC_W_FifoMode(&hi2c2, LSM303AGR_ACC_FM_BYPASS) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// ODR: Power Down
	if (LSM303AGR_ACC_W_ODR(&hi2c2, LSM303AGR_ACC_ODR_DO_PWR_DOWN) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// Operating mode Selection: High Resolution
	if (LSM303AGR_ACC_W_HiRes(&hi2c2, LSM303AGR_ACC_HR_ENABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	// Operating mode Selection: Low Power Disable
	if (LSM303AGR_ACC_W_LOWPWR_EN(&hi2c2, LSM303AGR_ACC_LPEN_DISABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	
	// Full Scale Selection
	if (LSM303AGR_ACC_W_FullScale(&hi2c2, LSM303AGR_ACC_FS_8G) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// Enable Axes: X, Y, Z
	if (LSM303AGR_ACC_W_XEN(&hi2c2, LSM303AGR_ACC_XEN_ENABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	if (LSM303AGR_ACC_W_YEN(&hi2c2, LSM303AGR_ACC_YEN_ENABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	if (LSM303AGR_ACC_W_ZEN(&hi2c2, LSM303AGR_ACC_ZEN_ENABLED) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	return status;
}

static Module_Status LSM303AccDeInit(void)
{
	// Check WhoAmI
	uint8_t who_am_i;
	if ( LSM303AGR_ACC_R_WHO_AM_I(&hi2c2, &who_am_i ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	if (who_am_i != LSM303AGR_ACC_WHO_AM_I)
		return H0BR4_ERR_LSM303;
	
	// ODR: Power Down
	if (LSM303AGR_ACC_W_ODR(&hi2c2, LSM303AGR_ACC_ODR_DO_PWR_DOWN) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	return H0BR4_OK;
}

static Module_Status LSM303AccGetAxis(int *accX, int *accY, int *accZ)
{
	int data[3];
  /* Read data from LSM303AGR. */
  if (!LSM303AGR_ACC_Get_Acceleration(&hi2c2, data)) {
    return H0BR4_ERR_LSM303;
  }
  /* Calculate the data. */
  *accX = data[0];
  *accX = data[1];
  *accZ = data[2];

  return H0BR4_OK;
}

static Module_Status LSM303AccGetAxesRaw(int16_t *accX, int16_t *accY, int16_t *accZ)
{
  Type3Axis16bit_U raw_data_tmp;
  uint8_t shift = 0;
  LSM303AGR_ACC_LPEN_t lp;
  LSM303AGR_ACC_HR_t hr;

  /* Determine which operational mode the acc is set */
  if (!LSM303AGR_ACC_R_HiRes(&hi2c2, &hr)) {
    return H0BR4_ERR_LSM303;
  }

  if (!LSM303AGR_ACC_R_LOWPWR_EN(&hi2c2, &lp)) {
    return H0BR4_ERR_LSM303;
  }

  if ((lp == LSM303AGR_ACC_LPEN_ENABLED) && (hr == LSM303AGR_ACC_HR_DISABLED)) {
    /* op mode is LP 8-bit */
    shift = 8;
  } else if ((lp == LSM303AGR_ACC_LPEN_DISABLED) && (hr == LSM303AGR_ACC_HR_DISABLED)) {
    /* op mode is Normal 10-bit */
    shift = 6;
  } else if ((lp == LSM303AGR_ACC_LPEN_DISABLED) && (hr == LSM303AGR_ACC_HR_ENABLED)) {
    /* op mode is HR 12-bit */
    shift = 4;
  } else {
    return H0BR4_ERR_LSM303;
  }

  /* Read output registers from LSM303AGR_ACC_GYRO_OUTX_L_XL to LSM303AGR_ACC_GYRO_OUTZ_H_XL. */
  if (!LSM303AGR_ACC_Get_Raw_Acceleration(&hi2c2, raw_data_tmp.u8bit )) {
    return H0BR4_ERR_LSM303;
  }

  /* Format the data. */
  *accX = (raw_data_tmp.i16bit[0] >> shift);
  *accY = (raw_data_tmp.i16bit[1] >> shift);
  *accZ = (raw_data_tmp.i16bit[2] >> shift);

  return H0BR4_OK;
}

static Module_Status LSM303AccGetDRDYStatus(bool *status)
{
  LSM303AGR_ACC_XDA_t status_raw;
  if (LSM303AGR_ACC_R_XDataAvail(&hi2c2, &status_raw ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
  switch (status_raw) {
    case LSM303AGR_ACC_XDA_AVAILABLE:
      *status = true;
      break;
    case LSM303AGR_ACC_XDA_NOT_AVAILABLE:
      *status = false;
      break;
    default:
      return H0BR4_ERR_LSM303;
  }
  return H0BR4_OK;
}

static Module_Status LSM303AccClearDRDY(void)
{
  uint8_t regValue[6];
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_X_L, regValue, 2) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_Y_L, regValue, 2) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_Z_L, regValue, 2) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
  return H0BR4_OK;
}


static Module_Status LSM303MagInit(void)
{
	// Check the Sensor
	uint8_t who_am_i = 0x00;
  if (LSM303AGR_MAG_R_WHO_AM_I(&hi2c2, &who_am_i ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
  if (who_am_i != LSM303AGR_MAG_WHO_AM_I) {
    return H0BR4_ERR_LSM303;
  }
	
	// Operating Mode: Power Down
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_IDLE1_MODE) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// Enable Block Data Update
  if (LSM303AGR_MAG_W_BDU(&hi2c2, LSM303AGR_MAG_BDU_ENABLED ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	
	// TODO: Change the default ODR
	if (LSM303AGR_MAG_W_ODR(&hi2c2, LSM303AGR_MAG_ODR_10Hz) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }

	// Self Test Disabled
	if (LSM303AGR_MAG_W_ST(&hi2c2, LSM303AGR_MAG_ST_DISABLED) == MEMS_ERROR ) {
    return H0BR4_ERR_LSM303;
  }

  return H0BR4_OK;
}

static Module_Status LSM303MagEnable(void)
{
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_CONTINUOS_MODE) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	return H0BR4_OK;
}

static Module_Status LSM303MagDisable(void)
{
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_IDLE1_MODE) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
	return H0BR4_OK;
}

static Module_Status LSM303MagDeInit(void)
{
	// Check the Sensor
	uint8_t who_am_i = 0x00;
  if (LSM303AGR_MAG_R_WHO_AM_I(&hi2c2, &who_am_i ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }
  if (who_am_i != LSM303AGR_MAG_WHO_AM_I) {
    return H0BR4_ERR_LSM303;
  }
	
	return LSM303MagDisable();
}

static Module_Status LSM303MagGetRawAxes(int16_t *magX, int16_t *magY, int16_t *magZ)
{
	int16_t *pData;
	uint8_t data[6];
	
	memset(data, 0, sizeof(data));
	
	if (LSM303AGR_MAG_Get_Raw_Magnetic(&hi2c2, data) == MEMS_ERROR) {
		return H0BR4_ERR_LSM303;
	}
	
	pData = (int16_t *)data;
	*magX = pData[0];
	*magY = pData[1];
	*magZ = pData[2];
	
	return H0BR4_OK;
}

static Module_Status LSM303MagGetAxes(int *magX, int *magY, int *magZ)
{
	Module_Status status = H0BR4_OK;
  int16_t rawMagX, rawMagY, rawMagZ;

  /* Read raw data from LSM303AGR output register. */
  if ((status = LSM303MagGetRawAxes(&rawMagX, &rawMagY, &rawMagZ)) != H0BR4_OK) {
    return status;
  }

  /* Set the raw data. */
  *magX = rawMagX * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
  *magY = rawMagY * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
  *magZ = rawMagZ * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;

  return status;
}

static Module_Status LSM303MagGetDRDYStatus(bool *status)
{
  LSM303AGR_MAG_ZYXDA_t status_raw;

  if (LSM303AGR_MAG_R_ZYXDA(&hi2c2, &status_raw ) == MEMS_ERROR) {
    return H0BR4_ERR_LSM303;
  }

  switch (status_raw) {
    case LSM303AGR_MAG_ZYXDA_EV_ON:
      *status = true;
      break;
    case LSM303AGR_MAG_ZYXDA_EV_OFF:
      *status = false;
      break;
    default:
      return H0BR4_ERR_LSM303;
  }
  return H0BR4_OK;
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
	int gyroX = 0, gyroY = 0, gyroZ = 0;
	
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (LSM6D3GetGyro(&gyroX, &gyroY, &gyroZ) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading gyro values\r\n");
		return pdFALSE;
	}
	
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "X:%d, Y:%d, Z:%d\r\n", gyroZ, gyroY, gyroZ);
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int accX = 0, accY = 0, accZ = 0;
	
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (LSM6D3GetAcc(&accX, &accY, &accZ) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading accelerometer values\r\n");
		return pdFALSE;
	}
	
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "X:%d, Y:%d, Z:%d\r\n", accX, accY, accZ);
	return pdFALSE;
}

static portBASE_TYPE LSM303GetMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	int magX = 0, magY = 0, magZ = 0;

	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (LSM303MagGetAxes(&magX, &magY, &magZ) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading magnetometer values\r\n");
		return pdFALSE;
	}
	
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "X:%d, Y:%d, Z:%d\r\n", magX, magY, magZ);
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something 
	*pcWriteBuffer = '\0';
	return pdFALSE;
}



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
