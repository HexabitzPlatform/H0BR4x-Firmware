/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved

 File Name     : H0BR4.c
 Description   : Source code for module H0BR4.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H0BR4_inputs.h"
#include "LSM6DS3.h"
#include "LSM303AGR_ACC.h"
#include "LSM303AGR_MAG.h"

#include <math.h>



#define LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G  1.5  /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */

#define MIN_MEMS_PERIOD_MS				200
#define MAX_MEMS_TIMEOUT_MS				0xFFFFFFFF
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;
extern I2C_HandleTypeDef hi2c2;
/* Module exported parameters ------------------------------------------------*/

float xGyro __attribute__((section(".mySection")));
float yGyro __attribute__((section(".mySection")));
float zGyro __attribute__((section(".mySection")));
float xAcc __attribute__((section(".mySection")));
float yAcc __attribute__((section(".mySection")));
float zAcc __attribute__((section(".mySection")));
int xMag __attribute__((section(".mySection")));
int yMag __attribute__((section(".mySection")));
int zMag __attribute__((section(".mySection")));
float temperature __attribute__((section(".mySection")));


module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr = NULL, .paramFormat =FMT_FLOAT, .paramName =""}};

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static Module_Status LSM6DS3Init(void);
static Module_Status LSM303MagInit(void);
static Module_Status LSM6DS3SampleGyroMDPS(int *gyroX,int *gyroY,int *gyroZ);
void ExecuteMonitor(void);

/* Create CLI commands --------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 ----------------------------------------------------------------------- 
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;

	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}
	
	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}
		
		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					}
				}
			}
		}
	}
	
	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
		}
	}
	
	HAL_FLASH_Lock();
	
	return result;
}

/* --- Clear array topology in SRAM and Flash RO --- 
 */
uint8_t ClearROtopology(void){
	// Clear the array 
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H0BR4 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	MEMS_GPIO_Init();
	MX_I2C_Init();
	LSM6DS3Init();
    LSM303MagInit();
	/* Create module special task (if needed) */
}
void initialValue(void)
{
	xGyro=0;
	yGyro=0;
	zGyro=0;
	xAcc=0;
	yAcc=0;
	zAcc=0;
	xMag=0;
	yMag=0;
	zMag=0;
	temperature=0;
}
/*-----------------------------------------------------------*/
/* --- H0BR4 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H0BR4_OK;


	switch(code){

		default:
			result =H0BR4_ERR_UnknownMessage;
			break;
	}
	
	return result;
}
/* --- Get the port for a given UART. 
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	else if(huart->Instance == USART6)
		return P6;
	
	return 0;
}

/*-----------------------------------------------------------*/
static Module_Status LSM6D3Enable(void){
	// Check WHO_AM_I Register
	uint8_t who_am_i =0;
	if(LSM6DS3_ACC_GYRO_R_WHO_AM_I(&hi2c2,&who_am_i) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	if(who_am_i != LSM6DS3_ACC_GYRO_WHO_AM_I)
		return H0BR4_ERR_LSM6DS3;

	// Enable register address automatically incremented during a multiple byte access with a serial interface
	if(LSM6DS3_ACC_GYRO_W_IF_Addr_Incr(&hi2c2,LSM6DS3_ACC_GYRO_IF_INC_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Bypass Mode
	if(LSM6DS3_ACC_GYRO_W_FIFO_MODE(&hi2c2,LSM6DS3_ACC_GYRO_FIFO_MODE_BYPASS) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	return H0BR4_OK;
}

static Module_Status LSM6D3SetupGyro(void){
	// Gyroscope ODR Init
	if(LSM6DS3_ACC_GYRO_W_ODR_G(&hi2c2,LSM6DS3_ACC_GYRO_ODR_G_13Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	// Gyroscope FS Init
	if(LSM6DS3_ACC_GYRO_W_FS_G(&hi2c2,LSM6DS3_ACC_GYRO_FS_G_2000dps) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Gyroscope Axes Status Init
	if(LSM6DS3_ACC_GYRO_W_XEN_G(&hi2c2,LSM6DS3_ACC_GYRO_XEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	if(LSM6DS3_ACC_GYRO_W_YEN_G(&hi2c2,LSM6DS3_ACC_GYRO_YEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	if(LSM6DS3_ACC_GYRO_W_ZEN_G(&hi2c2,LSM6DS3_ACC_GYRO_ZEN_G_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	return H0BR4_OK;
}

static Module_Status LSM6D3SetupAcc(void){
	// Accelerometer ODR Init
	if(LSM6DS3_ACC_GYRO_W_ODR_XL(&hi2c2,LSM6DS3_ACC_GYRO_ODR_XL_104Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Bandwidth Selection
	// Selection of bandwidth and ODR should be in accordance of Nyquist Sampling theorem!
	if(LSM6DS3_ACC_GYRO_W_BW_XL(&hi2c2,LSM6DS3_ACC_GYRO_BW_XL_50Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Accelerometer FS Init
	if(LSM6DS3_ACC_GYRO_W_FS_XL(&hi2c2,LSM6DS3_ACC_GYRO_FS_XL_16g) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Accelerometer Axes Status Init
	if(LSM6DS3_ACC_GYRO_W_XEN_XL(&hi2c2,LSM6DS3_ACC_GYRO_XEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	if(LSM6DS3_ACC_GYRO_W_YEN_XL(&hi2c2,LSM6DS3_ACC_GYRO_YEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	if(LSM6DS3_ACC_GYRO_W_ZEN_XL(&hi2c2,LSM6DS3_ACC_GYRO_ZEN_XL_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	// Enable Bandwidth Scaling
	if(LSM6DS3_ACC_GYRO_W_BW_Fixed_By_ODR(&hi2c2,LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED) != MEMS_ERROR)
		return H0BR4_ERR_LSM6DS3;

	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleGyroRaw(int16_t *gyroX,int16_t *gyroY,int16_t *gyroZ){
	uint8_t temp[6];
	if(LSM6DS3_ACC_GYRO_GetRawGyroData(&hi2c2,temp) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	*gyroX =concatBytes(temp[1],temp[0]);
	*gyroY =concatBytes(temp[3],temp[2]);
	*gyroZ =concatBytes(temp[5],temp[4]);

	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleGyroMDPS(int *gyroX,int *gyroY,int *gyroZ){
	int buff[3];
	if(LSM6DS3_ACC_Get_AngularRate(&hi2c2,buff,0) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	*gyroX =buff[0];
	*gyroY =buff[1];
	*gyroZ =buff[2];

	return H0BR4_OK;
}
/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

}

static Module_Status LSM6DS3Init(void){
	// Common Init
	Module_Status status =H0BR4_OK;
	if((status =LSM6D3Enable()) != H0BR4_OK)
		return status;

	if((status =LSM6D3SetupGyro()) != H0BR4_OK)
		return status;

	if((status =LSM6D3SetupAcc()) != H0BR4_OK)
		return status;

	// TODO: Configure Interrupt Lines

	return status;
}

static Module_Status LSM303MagEnable(void){
	if(LSM303AGR_MAG_W_MD(&hi2c2,LSM303AGR_MAG_MD_CONTINUOS_MODE) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	return H0BR4_OK;
}

static Module_Status LSM303MagInit(void){
	// Check the Sensor
	uint8_t who_am_i =0x00;
	if(LSM303AGR_MAG_R_WHO_AM_I(&hi2c2,&who_am_i) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	if(who_am_i != LSM303AGR_MAG_WHO_AM_I)
		return H0BR4_ERR_LSM303;

	// Operating Mode: Power Down
	if(LSM303AGR_MAG_W_MD(&hi2c2,LSM303AGR_MAG_MD_IDLE1_MODE) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	// Enable Block Data Update
	if(LSM303AGR_MAG_W_BDU(&hi2c2,LSM303AGR_MAG_BDU_ENABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	// TODO: Change the default ODR
	if(LSM303AGR_MAG_W_ODR(&hi2c2,LSM303AGR_MAG_ODR_10Hz) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	// Self Test Disabled
	if(LSM303AGR_MAG_W_ST(&hi2c2,LSM303AGR_MAG_ST_DISABLED) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	return LSM303MagEnable();
}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/

static Module_Status LSM6DS3SampleAccMG(int *accX,int *accY,int *accZ){
	int buff[3];
	if(LSM6DS3_ACC_Get_Acceleration(&hi2c2,buff,0) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	*accX =buff[0];
	*accY =buff[1];
	*accZ =buff[2];

	return H0BR4_OK;
}

static Module_Status LSM6DS3SampleTempCelsius(float *temp){
	uint8_t buff[2];
	if(LSM6DS3_ACC_GYRO_ReadReg(&hi2c2,LSM6DS3_ACC_GYRO_OUT_TEMP_L,buff,2) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM6DS3;

	int16_t rawTemp =concatBytes(buff[0],buff[1]);
	*temp =(((float )rawTemp) / 16) + 25;

	return H0BR4_OK;
}
static Module_Status LSM303SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ){
	int16_t *pData;
	uint8_t data[6];

	memset(data,0,sizeof(data));

	if(LSM303AGR_MAG_Get_Raw_Magnetic(&hi2c2,data) != MEMS_SUCCESS)
		return H0BR4_ERR_LSM303;

	pData =(int16_t* )data;
	*magX =pData[0];
	*magY =pData[1];
	*magZ =pData[2];

	return H0BR4_OK;
}

/*-----------------------------------------------------------*/

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */
Module_Status SampleGyroMDPS(int *gyroX,int *gyroY,int *gyroZ){
	return LSM6DS3SampleGyroMDPS(gyroX,gyroY,gyroZ);
}

Module_Status SampleAccG(float *x,float *y,float *z){
	Module_Status status =H0BR4_OK;
	int xInMG =0, yInMG =0, zInMG =0;

	if((status =LSM6DS3SampleAccMG(&xInMG,&yInMG,&zInMG)) != H0BR4_OK)
		return status;

	*x =((float )xInMG) / 1000;
	*y =((float )yInMG) / 1000;
	*z =((float )zInMG) / 1000;

	return status;
}
Module_Status SampleAccMG(int *accX,int *accY,int *accZ){
	return LSM6DS3SampleAccMG(accX,accY,accZ);
}
Module_Status SampleGyroDPS(float *x,float *y,float *z){
	Module_Status status =H0BR4_OK;
	int xInMDPS =0, yInMDPS =0, zInMDPS =0;

	if((status =LSM6DS3SampleGyroMDPS(&xInMDPS,&yInMDPS,&zInMDPS)) != H0BR4_OK)
		return status;

	*x =((float )xInMDPS) / 1000;
	*y =((float )yInMDPS) / 1000;
	*z =((float )zInMDPS) / 1000;

	return status;
}

Module_Status SampleTempCelsius(float *temp){
	return LSM6DS3SampleTempCelsius(temp);
}

Module_Status SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ){
	return LSM303SampleMagRaw(magX,magY,magZ);
}
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
