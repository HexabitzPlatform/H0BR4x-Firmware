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

#include "LSM6DS3.h"
#include "LSM303AGR_ACC.h"
#include "LSM303AGR_MAG.h"

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
static Module_Status LSM6DS3Init(void);
static Module_Status LSM303AccInit(void);
static Module_Status LSM303MagInit(void);

static Module_Status LSM6DS3SampleGyroMDPS(int *gyroX, int *gyroY, int *gyroZ);
static Module_Status LSM6DS3SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

static Module_Status LSM6DS3SampleAccMG(int *accX, int *accY, int *accZ);
static Module_Status LSM6DS3SampleAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ);

static Module_Status LSM6DS3SampleTempCelsius(float *temp);
static Module_Status LSM6DS3SampleTempFahrenheit(float *temp);

static Module_Status LSM303SampleMagMGauss(int *magX, int *magY, int *magZ);
static Module_Status LSM303SampleMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ);



/* Create CLI commands --------------------------------------------------------*/
static portBASE_TYPE LSM6DS3SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3SreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetGyroCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM303GetMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

const CLI_Command_Definition_t LSM6DS3SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "(H0BR4) sample:\r\n Syntax: sample [gyro]/[acc]/[mag]/[temp]\r\n \
		Get filtered and calibrated Gyro, Acc, Mag or Temp values in \
	dps, g, mguass or celsius units respectively.\r\n\r\n",
	LSM6DS3SampleSensorCommand,
	1
};

const CLI_Command_Definition_t LSM6DS3StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "(H0BR4) stream:\r\n Syntax: stream (period) (time) (port)/(buffer) [module]\r\n \
		Get stream of  filtered and calibrated Gyro, Acc, Mag or Temp values in \
	dps, g, mguass or celsius units respectively.\r\n\r\n",
	LSM6DS3SreamSensorCommand,
	1
};

const CLI_Command_Definition_t LSM6DS3GyroCommandDefinition = {
	(const int8_t *) "gyro",
	(const int8_t *) "(H0BR4) gyro:\r\n Get filtered and calibrated LSM6DS3 Gyro X,Y,Z values in dps.\r\n\r\n",
	LSM6DS3GetGyroCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3AccCommandDefinition = {
	(const int8_t *) "acc",
	(const int8_t *) "(H0BR4) acc:\r\n Get filtered and calibrated LSM6DS3 Accelerometer X,Y,Z values in g.\r\n\r\n",
	LSM6DS3GetAccCommand,
	0
};

const CLI_Command_Definition_t LSM303MagCommandDefinition = {
	(const int8_t *) "mag",
	(const int8_t *) "(H0BR4) mag:\r\n Get filtered and calibrated LSM6DS3 Magnetic strength X,Y,Z values in mGauss.\r\n\r\n",
	LSM303GetMagCommand,
	0
};

const CLI_Command_Definition_t LSM6DS3TempCommandDefinition = {
	(const int8_t *) "temp",
	(const int8_t *) "(H0BR4) temp:\r\n Get filtered and calibrated LSM6DS3 Temperature values in Celsius.\r\n\r\n",
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
	
	LSM6DS3Init();
	LSM303MagInit();
	
	// Disabling Accelerometer of LSM303AGR
	// LSM303AccInit();

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
			if ((result = SampleGyroDPSToPort(port, dst)) != H0BR4_OK)
				break;
			
			break;
		}
		case CODE_H0BR4_GET_ACC:
		{
			if ((result = SampleAccGToPort(port, dst)) != H0BR4_OK)
				break;
			
			break;
		}
		case CODE_H0BR4_GET_MAG:
		{
			if ((result = SampleMagMGaussToPort(port, dst)) != H0BR4_OK)
				break;
			
			break;
		}
		case CODE_H0BR4_GET_TEMP:
		{
			if ((result = SampleTempCToPort(port, dst)) != H0BR4_OK)
				break;
			
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
	FreeRTOS_CLIRegisterCommand(&LSM6DS3SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&LSM6DS3StreamCommandDefinition);
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
	// Check WHO_AM_I Register	
	uint8_t who_am_i = 0;
	if (LSM6DS3_ACC_GYRO_R_WHO_AM_I(&hi2c2, &who_am_i) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	if (who_am_i != LSM6DS3_ACC_GYRO_WHO_AM_I)
		return H0BR4_ERR_LSM6DS3;
	
	// Enable register address automatically incremented during a multiple byte access with a serial interface 
	if (LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(&hi2c2, LSM6DS3_ACC_GYRO_IF_INC_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Bypass Mode
	if (LSM6DS3_ACC_GYRO_W_FIFO_MODE(&hi2c2, LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	return H0BR4_OK;
}

static Module_Status LSM6D3SetupGyro(void)
{
	// Gyroscope ODR Init
	if (LSM6DS3_ACC_GYRO_W_ODR_G(&hi2c2, LSM6DS3_ACC_GYRO_ODR_G_13Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;
	
	// Gyroscope FS Init
	if (LSM6DS3_ACC_GYRO_W_FS_G(&hi2c2, LSM6DS3_ACC_GYRO_FS_G_2000dps) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Gyroscope Axes Status Init
	if (LSM6DS3_ACC_GYRO_W_XEN_G(&hi2c2, LSM6DS3_ACC_GYRO_XEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_YEN_G(&hi2c2, LSM6DS3_ACC_GYRO_YEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_ZEN_G(&hi2c2, LSM6DS3_ACC_GYRO_ZEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	return H0BR4_OK;
}

static Module_Status LSM6D3SetupAcc(void)
{
	// Accelerometer ODR Init
	if (LSM6DS3_ACC_GYRO_W_ODR_XL(&hi2c2, LSM6DS3_ACC_GYRO_ODR_XL_104Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Bandwidth Selection
	// Selection of bandwidth and ODR should be in accordance of Nyquist Sampling theorem!
	if (LSM6DS3_ACC_GYRO_W_BW_XL(&hi2c2, LSM6DS3_ACC_GYRO_BW_XL_50Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Accelerometer FS Init
	if (LSM6DS3_ACC_GYRO_W_FS_XL(&hi2c2, LSM6DS3_ACC_GYRO_FS_XL_16g) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Accelerometer Axes Status Init
	if (LSM6DS3_ACC_GYRO_W_XEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_XEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	if (LSM6DS3_ACC_GYRO_W_YEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_YEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
		
	if (LSM6DS3_ACC_GYRO_W_ZEN_XL(&hi2c2, LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	// Enable Bandwidth Scaling
	if (LSM6DS3_ACC_GYRO_W_BW_Fixed_By_ODR(&hi2c2, LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED) != MEMS_ERROR)
		return H0BR4_ERR_LSM6DS3;
	
	return H0BR4_OK;
}



static Module_Status LSM6DS3SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
	uint8_t temp[6];
	if (LSM6DS3_ACC_GYRO_GetRawGyroData(&hi2c2, temp) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	*gyroX = concatBytes(temp[1], temp[0]);
	*gyroY = concatBytes(temp[3], temp[2]);
	*gyroZ = concatBytes(temp[5], temp[4]);
	
	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleGyroMDPS(int *gyroX, int *gyroY, int *gyroZ)
{
	int buff[3];
	if (LSM6DS3_ACC_Get_AngularRate(&hi2c2, buff, 0) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	*gyroX = buff[0];
	*gyroY = buff[1];
	*gyroZ = buff[2];
	
	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ)
{
	uint8_t temp[6];
	if (LSM6DS3_ACC_GYRO_GetRawAccData(&hi2c2, temp) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	*accX = concatBytes(temp[1], temp[0]);
	*accY = concatBytes(temp[3], temp[2]);
	*accZ = concatBytes(temp[5], temp[4]);
	
	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleAccMG(int *accX, int *accY, int *accZ)
{
	int buff[3];
	if (LSM6DS3_ACC_Get_Acceleration(&hi2c2, buff, 0) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	*accX = buff[0];
	*accY = buff[1];
	*accZ = buff[2];
	
	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleTempCelsius(float *temp)
{
	uint8_t buff[2];
	if (LSM6DS3_ACC_GYRO_ReadReg(&hi2c2, LSM6DS3_ACC_GYRO_OUT_TEMP_L, buff, 2) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;
	
	int16_t rawTemp = concatBytes(buff[0], buff[1]);
	*temp = (((float)rawTemp)/16) + 25;
	
	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleTempFahrenheit(float *temp)
{
	Module_Status status = H0BR4_OK;
	float celsius = 0;
	
	if ((status = LSM6DS3SampleTempCelsius(&celsius)) != H0BR4_OK)
		return status;
	
	*temp = celsiusToFahrenheit(celsius);
	return H0BR4_OK;
}

static Module_Status LSM6DS3Init(void)
{
	// Common Init
	Module_Status status = H0BR4_OK;
	if ((status = LSM6D3Enable()) != H0BR4_OK)
		return status;
	
	if ((status = LSM6D3SetupGyro()) != H0BR4_OK)
		return status;
	
	if ((status = LSM6D3SetupAcc()) != H0BR4_OK)
		return status;
	
	// TODO: Configure Interrupt Lines
	
	return status;
}


static Module_Status LSM303AccInit(void)
{
	// Check WhoAmI
	uint8_t who_am_i;
	if (LSM303AGR_ACC_R_WHO_AM_I(&hi2c2, &who_am_i ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

	if (who_am_i != LSM303AGR_ACC_WHO_AM_I)
		return H0BR4_ERR_LSM303;
	
	// Enable Block Data update
	if (LSM303AGR_ACC_W_BlockDataUpdate(&hi2c2, LSM303AGR_ACC_BDU_ENABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// FIFO Mode: ByPass
	if (LSM303AGR_ACC_W_FifoMode(&hi2c2, LSM303AGR_ACC_FM_BYPASS) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// ODR: Power Down
	if (LSM303AGR_ACC_W_ODR(&hi2c2, LSM303AGR_ACC_ODR_DO_PWR_DOWN) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// Operating mode Selection: High Resolution
	if (LSM303AGR_ACC_W_HiRes(&hi2c2, LSM303AGR_ACC_HR_ENABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// Operating mode Selection: Low Power Disable
	if (LSM303AGR_ACC_W_LOWPWR_EN(&hi2c2, LSM303AGR_ACC_LPEN_DISABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// Full Scale Selection
	if (LSM303AGR_ACC_W_FullScale(&hi2c2, LSM303AGR_ACC_FS_8G) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// Enable Axes: X, Y, Z
	if (LSM303AGR_ACC_W_XEN(&hi2c2, LSM303AGR_ACC_XEN_ENABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	if (LSM303AGR_ACC_W_YEN(&hi2c2, LSM303AGR_ACC_YEN_ENABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	if (LSM303AGR_ACC_W_ZEN(&hi2c2, LSM303AGR_ACC_ZEN_ENABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	return H0BR4_OK;
}

static Module_Status LSM303AccDeInit(void)
{
	// Check WhoAmI
	uint8_t who_am_i;
	if ( LSM303AGR_ACC_R_WHO_AM_I(&hi2c2, &who_am_i ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
  
	if (who_am_i != LSM303AGR_ACC_WHO_AM_I)
		return H0BR4_ERR_LSM303;
	
	// ODR: Power Down
	if (LSM303AGR_ACC_W_ODR(&hi2c2, LSM303AGR_ACC_ODR_DO_PWR_DOWN) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	return H0BR4_OK;
}

static Module_Status LSM303AccGetAxis(int *accX, int *accY, int *accZ)
{
	int data[3];
  /* Read data from LSM303AGR. */
  if (LSM303AGR_ACC_Get_Acceleration(&hi2c2, data) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
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
  if (LSM303AGR_ACC_R_HiRes(&hi2c2, &hr) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

  if (LSM303AGR_ACC_R_LOWPWR_EN(&hi2c2, &lp) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

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
  if (LSM303AGR_ACC_Get_Raw_Acceleration(&hi2c2, raw_data_tmp.u8bit) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

  /* Format the data. */
  *accX = (raw_data_tmp.i16bit[0] >> shift);
  *accY = (raw_data_tmp.i16bit[1] >> shift);
  *accZ = (raw_data_tmp.i16bit[2] >> shift);

  return H0BR4_OK;
}

static Module_Status LSM303AccGetDRDYStatus(bool *status)
{
  LSM303AGR_ACC_XDA_t status_raw;
  if (LSM303AGR_ACC_R_XDataAvail(&hi2c2, &status_raw ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
  
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
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_X_L, regValue, 2) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
  
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_Y_L, regValue, 2) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	if (LSM303AGR_ACC_ReadReg(&hi2c2, LSM303AGR_ACC_OUT_Z_L, regValue, 2) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
  return H0BR4_OK;
}

static Module_Status LSM303MagEnable(void)
{
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_CONTINUOS_MODE) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	return H0BR4_OK;
}

static Module_Status LSM303MagDisable(void)
{
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_IDLE1_MODE) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	return H0BR4_OK;
}

static Module_Status LSM303MagInit(void)
{
	// Check the Sensor
	uint8_t who_am_i = 0x00;
  if (LSM303AGR_MAG_R_WHO_AM_I(&hi2c2, &who_am_i ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
  
  if (who_am_i != LSM303AGR_MAG_WHO_AM_I)
    return H0BR4_ERR_LSM303;
	
	// Operating Mode: Power Down
	if (LSM303AGR_MAG_W_MD(&hi2c2, LSM303AGR_MAG_MD_IDLE1_MODE) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// Enable Block Data Update
  if (LSM303AGR_MAG_W_BDU(&hi2c2, LSM303AGR_MAG_BDU_ENABLED ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
	// TODO: Change the default ODR
	if (LSM303AGR_MAG_W_ODR(&hi2c2, LSM303AGR_MAG_ODR_10Hz) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

	// Self Test Disabled
	if (LSM303AGR_MAG_W_ST(&hi2c2, LSM303AGR_MAG_ST_DISABLED) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

  return LSM303MagEnable();
}

static Module_Status LSM303MagDeInit(void)
{
	// Check the Sensor
	uint8_t who_am_i = 0x00;
  if (LSM303AGR_MAG_R_WHO_AM_I(&hi2c2, &who_am_i ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;
	
  if (who_am_i != LSM303AGR_MAG_WHO_AM_I)
    return H0BR4_ERR_LSM303;
	
	return LSM303MagDisable();
}

static Module_Status LSM303SampleMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ)
{
	int16_t *pData;
	uint8_t data[6];
	
	memset(data, 0, sizeof(data));
	
	if (LSM303AGR_MAG_Get_Raw_Magnetic(&hi2c2, data) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;
	
	pData = (int16_t *)data;
	*magX = pData[0];
	*magY = pData[1];
	*magZ = pData[2];
	
	return H0BR4_OK;
}

static Module_Status LSM303SampleMagMGauss(int *magX, int *magY, int *magZ)
{
	Module_Status status = H0BR4_OK;
  int16_t rawMagX, rawMagY, rawMagZ;

  /* Read raw data from LSM303AGR output register. */
  if ((status = LSM303SampleMagRaw(&rawMagX, &rawMagY, &rawMagZ)) != H0BR4_OK)
    return status;

  /* Set the raw data. */
  *magX = rawMagX * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
  *magY = rawMagY * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
  *magZ = rawMagZ * (float)LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
  return status;
}

static Module_Status LSM303MagGetDRDYStatus(bool *status)
{
  LSM303AGR_MAG_ZYXDA_t status_raw;

  if (LSM303AGR_MAG_R_ZYXDA(&hi2c2, &status_raw ) != MEMS_SUCCESS)
    return H0BR4_ERR_LSM303;

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

Module_Status SampleGyroMDPS(int *gyroX, int *gyroY, int *gyroZ)
{
	return LSM6DS3SampleGyroMDPS(gyroX, gyroY, gyroZ);
}

Module_Status SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ)
{
	return LSM6DS3SampleGyroRaw(gyroX, gyroY, gyroZ);
}


Module_Status SampleGyroDPSToPort(uint8_t port, uint8_t module)
{
	float buffer[3]; // Three Samples X, Y, Z
	Module_Status status = H0BR4_OK;
	
	if ((status = SampleGyroDPSToBuf(buffer)) != H0BR4_OK)
		return status;
	
	memcpy(messageParams, buffer, sizeof(buffer));
	SendMessageFromPort(port, myID, module, CODE_H0BR4_RESULT_GYRO, sizeof(buffer));
	return status;
}

Module_Status SampleGyroDPSToString(char *cstring, size_t maxLen)
{
	Module_Status status = H0BR4_OK;
	float x = 0, y = 0, z = 0;
	
	if ((status = SampleGyroDPS(&x, &y, &z)) != H0BR4_OK)
		return status;
	
	snprintf(cstring, maxLen, "Gyro(DPS) | X: %.2f, Y: %.2f, Z: %.2f\r\n", x, y, z);
	return status;
}

Module_Status SampleGyroDPS(float *x, float *y, float *z)
{
	Module_Status status = H0BR4_OK;
	int xInMDPS = 0, yInMDPS = 0, zInMDPS = 0;
	
	if ((status = LSM6DS3SampleGyroMDPS(&xInMDPS, &yInMDPS, &zInMDPS)) != H0BR4_OK)
		return status;
	
	*x = ((float)xInMDPS) / 1000;
	*y = ((float)yInMDPS) / 1000;
	*z = ((float)zInMDPS) / 1000;
	
	return status;
}

Module_Status SampleGyroDPSToBuf(float *buffer)
{
	return SampleGyroDPS(buffer, buffer + 1, buffer + 2);
}

Module_Status SampleAccMG(int *accX, int *accY, int *accZ)
{
	return LSM6DS3SampleAccMG(accX, accY, accZ);
}

Module_Status SampleAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ)
{
	return LSM6DS3SampleAccRaw(accX, accY, accZ);
}

Module_Status SampleAccGToPort(uint8_t port, uint8_t module)
{
	float buffer[3]; // Three Samples X, Y, Z
	Module_Status status = H0BR4_OK;
	
	if ((status = SampleAccGToBuf(buffer)) != H0BR4_OK)
		return status;
	
	memcpy(messageParams, buffer, sizeof(buffer));
	SendMessageFromPort(port, myID, module, CODE_H0BR4_RESULT_ACC, sizeof(buffer));
	return status;
}

Module_Status SampleAccGToString(char *cstring, size_t maxLen)
{
	Module_Status status = H0BR4_OK;
	float x = 0, y = 0, z = 0;
	
	if ((status = SampleAccG(&x, &y, &z)) != H0BR4_OK)
		return status;
	
	snprintf(cstring, maxLen, "Acc(G) | X: %.2f, Y: %.2f, Z: %.2f\r\n", x, y, z);
	return status;
}

Module_Status SampleAccG(float *x, float *y, float *z)
{
	Module_Status status = H0BR4_OK;
	int xInMG = 0, yInMG = 0, zInMG = 0;
	
	if ((status = LSM6DS3SampleAccMG(&xInMG, &yInMG, &zInMG)) != H0BR4_OK)
		return status;
	
	*x = ((float)xInMG) / 1000;
	*y = ((float)yInMG) / 1000;
	*z = ((float)zInMG) / 1000;
	
	return status;
}

Module_Status SampleAccGToBuf(float *buffer)
{
	return SampleAccG(buffer, buffer + 1, buffer + 2);
}

Module_Status SampleMagMGauss(int *magX, int *magY, int *magZ)
{
	return LSM303SampleMagMGauss(magX, magY, magZ);
}

Module_Status SampleMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ)
{
	return LSM303SampleMagRaw(magX, magY, magZ);
}

Module_Status SampleMagMGaussToPort(uint8_t port, uint8_t module)
{
	int buffer[3]; // Three Samples X, Y, Z
	Module_Status status = H0BR4_OK;
	
	if ((status = SampleMagMGaussToBuf(buffer)) != H0BR4_OK)
		return status;
	
	memcpy(messageParams, buffer, sizeof(buffer));
	SendMessageFromPort(port, myID, module, CODE_H0BR4_RESULT_MAG, sizeof(buffer));
	return status;
}

Module_Status SampleMagMGaussToString(char *cstring, size_t maxLen)
{
	Module_Status status = H0BR4_OK;
	int x = 0, y = 0, z = 0;
	
	if ((status = LSM303SampleMagMGauss(&x, &y, &z)) != H0BR4_OK)
		return status;
	
	snprintf(cstring, maxLen, "Mag(mGauss) | X: %d, Y: %d, Z: %d\r\n", x, y, z);
	return status;
}

Module_Status SampleMagMGaussToBuf(int *buffer)
{
	return LSM303SampleMagMGauss(buffer, buffer + 1, buffer + 2);
}

Module_Status SampleTempCelsius(float *temp)
{
	return LSM6DS3SampleTempCelsius(temp);
}

Module_Status SampleTempFahrenheit(float *temp)
{
	return LSM6DS3SampleTempFahrenheit(temp);
}

Module_Status SampleTempCToPort(uint8_t port, uint8_t module)
{
	float temp;
	Module_Status status = H0BR4_OK;
	
	if ((status = LSM6DS3SampleTempCelsius(&temp)) != H0BR4_OK)
		return status;
	
	memcpy(messageParams, &temp, sizeof(temp));
	SendMessageFromPort(port, myID, module, CODE_H0BR4_RESULT_TEMP, sizeof(temp));
	return status;
}

Module_Status SampleTempCToString(char *cstring, size_t maxLen)
{
	Module_Status status = H0BR4_OK;
	float temp;
	
	if ((status = LSM6DS3SampleTempCelsius(&temp)) != H0BR4_OK)
		return status;
	
	snprintf(cstring, maxLen, "Temp(Celsius): %0.2f\r\n", temp);
	return status;
}



/* -----------------------------------------------------------------------
	|															Commands																 	|
   ----------------------------------------------------------------------- 
*/

static portBASE_TYPE LSM6DS3SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// int x = 0, y = 0, z = 0;
	// float temp = 25.0;
	
	const char *const gyroCmdName = "gyro";
	const char *const accCmdName = "acc";
	const char *const magCmdName = "mag";
	const char *const tempCmdName = "temp";
	
	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen = 0;
	
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	pSensName = (const char *)FreeRTOS_CLIGetParameter(pcCommandString, 1, &sensNameLen);
	
	if (pSensName == NULL) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		return pdFALSE;
	}
	
	do {
		if (!strncmp(pSensName, gyroCmdName, strlen(gyroCmdName))) {
			if (SampleGyroDPSToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK)
				break;
			
		} else if (!strncmp(pSensName, accCmdName, strlen(accCmdName))) {
			if (SampleAccGToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK)
				break;
		
		} else if (!strncmp(pSensName, magCmdName, strlen(magCmdName))) {
			if (SampleMagMGaussToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK)
				break;
			
		} else if (!strncmp(pSensName, tempCmdName, strlen(tempCmdName))) {
			if (SampleTempCToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK)
				break;
			
		} else {
			snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Invalid Arguments\r\n");
		}
	
		return pdFALSE;
	} while (0);
	
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading %s values\r\n", pSensName);
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3SreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// int x = 0, y = 0, z = 0;
	
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Command not supported\r\n");
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetGyroCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (SampleGyroDPSToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading gyro values\r\n");
		return pdFALSE;
	}
	
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetAccCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (SampleAccGToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading accelerometer values\r\n");
		return pdFALSE;
	}
	
	return pdFALSE;
}

static portBASE_TYPE LSM303GetMagCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (SampleMagMGaussToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading magnetometer values\r\n");
		return pdFALSE;
	}
	return pdFALSE;
}

static portBASE_TYPE LSM6DS3GetTempCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	// Make sure we return something
	*pcWriteBuffer = '\0';
	
	if (SampleTempCToString((char *)pcWriteBuffer, xWriteBufferLen) != H0BR4_OK) {
		snprintf((char *)pcWriteBuffer, xWriteBufferLen, "Error reading temperature value\r\n");
		return pdFALSE;
	}
	
	return pdFALSE;
}



/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
