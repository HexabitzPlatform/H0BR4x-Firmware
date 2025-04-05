/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0BR4.c
 Description   : Source code for module H0BR4.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H0BR4_inputs.h"
#include "LSM6DS3TR_C_APIS.h"
#include "LSM303AGR_APIS.h"
#include <math.h>

/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

extern stmdev_ctx_t dev_ctx;

/* Private Variables *******************************************************/

/* Stream to port variables */
uint32_t PortNumOfSamples = 0u;    /* Number of samples for port streaming */
uint32_t PortSamples = 0u;         /* Current sample count for port (if needed separately) */
uint8_t PortModule = 0u;           /* Module ID for port streaming */
uint8_t PortNumber = 0u;           /* Port number for streaming */
All_Data PortFunction;                    /* Function pointer or struct for port streaming */

/* Stream to terminal variables */
uint32_t TerminalNumOfSamples = 0u; /* Number of samples for terminal streaming */
uint8_t TerminalPort = 0u;          /* Port number for terminal streaming */
All_Data TerminalFunction;                   /* Function pointer or struct for terminal streaming */
uint32_t TerminalTimeout = 0u;               /* Timeout value for terminal streaming */
uint8_t StreamMode = 0u;                     /* Streaming mode selector (port or terminal) */
uint8_t StopeCliStreamFlag = 0u;             /* Flag to stop CLI streaming */
/* General streaming variable */
uint32_t SampleCount = 0u;                   /* Total sample counter */


static bool stopStream = false;

float H0BR4_gyroX =0.0f;
float H0BR4_gyroY =0.0f;
float H0BR4_gyroZ =0.0f;
float H0BR4_accX =0.0f;
float H0BR4_accY =0.0f;
float H0BR4_accZ =0.0f;
float H0BR4_temp =0.0f;
int H0BR4_magX =0.0f;
int H0BR4_magY =0.0f;
int H0BR4_magZ =0.0f;


TaskHandle_t IMU_TaskTaskHandle = NULL;
TimerHandle_t xTimerStream = NULL;

/* Exported Typedef */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] ={
	{.ParamPtr =&H0BR4_gyroX, .ParamFormat =FMT_FLOAT, .ParamName ="gyrox"},
	{.ParamPtr =&H0BR4_gyroY, .ParamFormat =FMT_FLOAT, .ParamName ="gyroy"},
	{.ParamPtr =&H0BR4_gyroZ, .ParamFormat =FMT_FLOAT, .ParamName ="gyroz"},
	{.ParamPtr =&H0BR4_accX, .ParamFormat =FMT_FLOAT, .ParamName ="accx"},
	{.ParamPtr =&H0BR4_accY, .ParamFormat =FMT_FLOAT, .ParamName ="accy"},
	{.ParamPtr =&H0BR4_accZ, .ParamFormat =FMT_FLOAT, .ParamName ="accz"},
	{.ParamPtr =&H0BR4_magX, .ParamFormat =FMT_INT32, .ParamName ="magx"},
	{.ParamPtr =&H0BR4_magY, .ParamFormat =FMT_INT32, .ParamName ="magy"},
	{.ParamPtr =&H0BR4_magZ, .ParamFormat =FMT_INT32, .ParamName ="magz"},
	{.ParamPtr =&H0BR4_temp, .ParamFormat =FMT_FLOAT, .ParamName ="temp"}, };

/* Private function prototypes *********************************************/
uint8_t ClearROtopology(void);
void Module_Peripheral_Init(void);
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift);

/* Local function prototypes ***********************************************/
void StreamTimeCallback(TimerHandle_t xTimerStream);

void SampleGyroDPSToString(char *cstring,size_t maxLen);
void SampleAccGToString(char *cstring,size_t maxLen);
void SampleMagMGaussToString(char *cstring,size_t maxLen);
void SampleTempCToString(char *cstring,size_t maxLen);

void SampleTempBuff(float *buffer);
void SampleMagBuf(float *buffer);
void SampleAccBuf(float *buffer);
void SampleGyroBuf(float *buffer);

void FLASH_Page_Eras(uint32_t Addr);
void ExecuteMonitor(void);

typedef void (*SampleToString)(char*,size_t);
typedef void (*SampleMemsToBuffer)(float *buffer);

Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction);
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples);
static Module_Status StreamToCLI(uint32_t Numofsamples,uint32_t timeout,SampleToString function);
static Module_Status StreamMemsToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleMemsToBuffer function);


/* Create CLI commands *****************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : sample */
const CLI_Command_Definition_t SampleCommandDefinition = {
	(const int8_t *) "sample",
	(const int8_t *) "sample:\r\n Syntax: sample [Gyro]/[Acc]/[Mag]/[Temp].\r\n\r\n",
	SampleSensorCommand,
	1
};

/***************************************************************************/
/* CLI command structure : stream */
const CLI_Command_Definition_t StreamCommandDefinition = {
	(const int8_t *) "stream",
	(const int8_t *) "stream:\r\n Syntax: stream [Gyro]/[Acc]/[Mag]/[Temp] ( Numofsamples ) (timeout) .\r\n\r\n",
	StreamSensorCommand,
	-1
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/

/*
 * @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
    RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
    RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void){

	HAL_StatusTypeDef flashStatus =HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd =8;
	uint16_t temp =0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2,TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(flashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if(myID){

		/* Save module's ID */
		temp =(uint16_t )(N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS,temp);

		/* Wait for a Write operation to complete */
		flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

		if(flashStatus != HAL_OK){
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t row =1; row <= N; row++){
			for(uint8_t column =0; column <= MaxNumOfPorts; column++){
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if(array[row - 1][0]){
					/* Save each element in topology array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,TOPOLOGY_START_ADDRESS + flashAdd,array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(flashStatus != HAL_OK){
						/* return FLASH error code */
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd +=8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void){
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] ={0};

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2,SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);

	if(FlashStatus != HAL_OK){
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for(uint8_t index =0; index < numOfRecordedSnippets; index++){
		/* Check if Snippet condition is true or false */
		if(Snippets[index].Condition.ConditionType){
			/* A marker to separate Snippets */
			snipBuffer[0] =0xFE;
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&Snippets[index],sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for(uint8_t j =0; j < (sizeof(Snippet_t) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j * 8]);
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for(uint8_t j =0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(Snippets[index].CMD + j * 4));
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void){
	/* Clear the array */
	memset(array,0,sizeof(array));
	N =1;
	myID =0;
	
	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
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

/***************************************************************************/
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

/***************************************************************************/
/* H0BR4 module initialization */
void Module_Peripheral_Init(void){

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	GPIO_Init();
	MEMS_GPIO_Init();
	MX_I2C_Init();
	LSM6DS3TR_C_Init();
	LSM303MagInit();

	/* Circulating DMA Channels ON All Module */
	for(int i =1; i <= NumOfPorts; i++){
		if(GetUart(i) == &huart1){
			index_dma[i - 1] =&(DMA1_Channel1->CNDTR);
		}
		else if(GetUart(i) == &huart2){
			index_dma[i - 1] =&(DMA1_Channel2->CNDTR);
		}
		else if(GetUart(i) == &huart3){
			index_dma[i - 1] =&(DMA1_Channel3->CNDTR);
		}
		else if(GetUart(i) == &huart4){
			index_dma[i - 1] =&(DMA1_Channel4->CNDTR);
		}
		else if(GetUart(i) == &huart5){
			index_dma[i - 1] =&(DMA1_Channel5->CNDTR);
		}
		else if(GetUart(i) == &huart6){
			index_dma[i - 1] =&(DMA1_Channel6->CNDTR);
		}
	}
	/* Create a timeout software timer StreamSamplsToPort() API */
	xTimerStream =xTimerCreate("StreamTimer",pdMS_TO_TICKS(1000),pdTRUE,(void* )1,StreamTimeCallback);
}

/***************************************************************************/
/* H0BR4 message processing task */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H0BR4_OK;
	uint32_t period =0, timeout =0;

	switch(code){
		case CODE_H0BR4_SAMPLE_GYRO: {
			SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],GYRO);
			break;
		}
		case CODE_H0BR4_SAMPLE_ACC: {
			SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],ACC);
			break;
		}
		case CODE_H0BR4_SAMPLE_MAG: {
			SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],MAG);
			break;
		}
		case CODE_H0BR4_SAMPLE_TEMP: {
			SampleToPort(cMessage[port - 1][shift],cMessage[port - 1][1 + shift],TEMP);
			break;
		}

		default:
			result =H0BR4_ERR_UnknownMessage;
			break;
	}
	
	return result;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void){
	FreeRTOS_CLIRegisterCommand(&SampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&StreamCommandDefinition);
}

/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P6;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	else if(huart->Instance == USART6)
		return P3;
	
	return 0;
}

/***************************************************************************/
/* This functions is useful only for input (sensors) modules.
 * @brief: Samples a module parameter value based on parameter index.
 * @param paramIndex: Index of the parameter (1-based index).
 * @param value: Pointer to store the sampled float value.
 * @retval: Module_Status indicating success or failure.
 */
Module_Status GetModuleParameter(uint8_t paramIndex,float *value){
	Module_Status status =BOS_OK;

	switch(paramIndex){
		/* Sample gyroX */
		case 1:
			status =SampleGyroDPS(value,NULL,NULL);
			break;

		/* Sample gyroY */
		case 2:
			status =SampleGyroDPS(NULL,value,NULL);
			break;

		/* Sample gyroZ */
		case 3:
			status =SampleGyroDPS(NULL,NULL,value);
			break;

		/* Sample accX */
		case 4:
			status =SampleAccG(value,NULL,NULL);
			break;

		/* Sample accY */
		case 5:
			status =SampleAccG(NULL,value,NULL);
			break;

		/* Sample accZ */
		case 6:
			status =SampleAccG(NULL,NULL,value);
			break;

		/* Sample magX (convert int to float) */
		case 7: {
			int temp =0;
			status =SampleMagMGauss(&temp,NULL,NULL);
			if(status == BOS_OK)
				*value =(float )temp;
			break;
		}

		/* Sample magY (convert int to float) */
		case 8: {
			int temp =0;
			status =SampleMagMGauss(NULL,&temp,NULL);
			if(status == BOS_OK)
				*value =(float )temp;
			break;
		}

		/* Sample magZ (convert int to float) */
		case 9: {
			int temp =0;
			status =SampleMagMGauss(NULL,NULL,&temp);
			if(status == BOS_OK)
				*value =(float )temp;
			break;
		}

		/* Sample temperature in Celsius */
		case 10:
			status =SampleTempCelsius(value);
			break;

		/* Invalid parameter index */
		default:
			status =BOS_ERR_WrongParam;
			break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/**
 * @brief: Streams MEMS sensor data to a buffer.
 * @param1: Pointer to the buffer where data will be stored.
 * @param2: Number of samples to take.
 * @param3: Timeout period for the operation.
 * @param4: Function pointer to the sampling function (e.g., SampleAccBuf, SampleGyroBuf).
 * @retval: Module status indicating success or error.
 */
static Module_Status StreamMemsToBuf(float *buffer,uint32_t Numofsamples,uint32_t timeout,SampleMemsToBuffer function){
	Module_Status status =H0BR4_OK;
	uint16_t StreamIndex =0;
	uint32_t period =timeout / Numofsamples;

	/* Check if the calculated period is valid */
	if(period < MIN_MEMS_PERIOD_MS)
		return H0BR4_ERR_WrongParams;

	timeout =period;
	long numTimes =timeout / period;
	stopStream = false;

	/* Stream data to buffer */
	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		if(function == SampleTempBuff){
			float sample;
			function(&sample);
			buffer[StreamIndex] =sample;
			StreamIndex++;
		}
		else{
			float Axis[3];
			function(Axis);
			buffer[StreamIndex] =Axis[0];
			buffer[StreamIndex + 1] =Axis[1];
			buffer[StreamIndex + 2] =Axis[2];
			StreamIndex +=3;
		}

		/* Delay for the specified period */
		vTaskDelay(pdMS_TO_TICKS(period));

		/* Check if streaming should be stopped */
		if(stopStream){
			status =H0BR4_ERR_TERMINATED;
			break;
		}
	}

	return status;
}

/***************************************************************************/
/*
 * @brief: Samples accelerometer data into a buffer.
 * @param buffer: Pointer to the buffer where accelerometer data will be stored.
 * @retval: None
 */
void SampleAccBuf(float *buffer){
	float Acc[3];
	SampleAccG(Acc,Acc + 1,Acc + 2);
	buffer[0] =Acc[0];
	buffer[1] =Acc[1];
	buffer[2] =Acc[2];
}

/***************************************************************************/
/*
 * @brief: Samples gyroscope data into a buffer.
 * @param buffer: Pointer to the buffer where gyroscope data will be stored.
 * @retval: None
 */
void SampleGyroBuf(float *buffer){
	float Gyro[3];
	SampleGyroDPS(Gyro,Gyro + 1,Gyro + 2);
	buffer[0] =Gyro[0];
	buffer[1] =Gyro[1];
	buffer[2] =Gyro[2];
}

/***************************************************************************/
/*
 * @brief: Samples magnetometer data into a buffer.
 * @param buffer: Pointer to the buffer where magnetometer data will be stored.
 * @retval: None
 */
void SampleMagBuf(float *buffer){
	int Mag[3];
	SampleMagMGauss(Mag,Mag + 1,Mag + 2);
	buffer[0] =Mag[0];
	buffer[1] =Mag[1];
	buffer[2] =Mag[2];
}

/***************************************************************************/
/*
 * @brief: Samples temperature data into a buffer.
 * @param buffer: Pointer to the buffer where temperature data will be stored.
 * @retval: None
 */
void SampleTempBuff(float *buffer){
	float Temp;
	SampleTempCelsius(&Temp);
	*buffer =Temp;
}

/***************************************************************************/
/*
 * @brief: Streams MEMS sensor data to the CLI (Command Line Interface).
 * @param1: Number of samples to take.
 * @param2: Timeout period for the operation.
 * @param3: Function pointer to the sampling function (e.g., SampleAccGToString, SampleGyroDPSToString).
 * @retval: Module status indicating success or error.
 */
static Module_Status StreamToCLI(uint32_t Numofsamples,uint32_t timeout,SampleToString function){
	Module_Status status =H0BR4_OK; // Initialize status to OK
	int8_t *pcOutputString = NULL; // Pointer to output string
	uint32_t period =timeout / Numofsamples; // Calculate the period for each sample

	/* Check if the calculated period is valid */
	if(period < MIN_MEMS_PERIOD_MS)
		return H0BR4_ERR_WrongParams;

	/* Check if CLI is enabled */
	for(uint8_t chr =0; chr < MSG_RX_BUF_SIZE; chr++){
		if(UARTRxBuf[PcPort - 1][chr] == '\r'){
			UARTRxBuf[PcPort - 1][chr] =0; // Null-terminate the buffer
		}
	}

	/* Check if streaming should be stopped */
	if(1 == StopeCliStreamFlag){
		StopeCliStreamFlag =0;
		static char *pcOKMessage =(int8_t* )"Stop stream!\n\r"; // Message to indicate stopping stream
		writePxITMutex(PcPort,pcOKMessage,strlen(pcOKMessage),10); // Write the stop message to CLI
		return status;
	}

	/* Adjust timeout period if necessary */
	if(period > timeout)
		timeout =period;

	long numTimes =timeout / period;
	stopStream = false;

	/* Stream data to CLI */
	while((numTimes-- > 0) || (timeout >= MAX_MEMS_TIMEOUT_MS)){
		pcOutputString =FreeRTOS_CLIGetOutputBuffer(); // Get output buffer for CLI
		function((char* )pcOutputString,100); // Call the sampling function to get data
		writePxMutex(PcPort,(char* )pcOutputString,strlen((char* )pcOutputString),cmd500ms,HAL_MAX_DELAY); // Write data to CLI

		if(PollingSleepCLISafe(period,Numofsamples) != H0BR4_OK)
			break;
	}

	memset((char* )pcOutputString,0,configCOMMAND_INT_MAX_OUTPUT_SIZE); // Clear the output buffer
	sprintf((char* )pcOutputString,"\r\n"); // Add newline to output buffer

	return status; /* Return the status of the operation */
}

/***************************************************************************/
/*
 * @brief: Samples temperature data and converts it to a string.
 * @param cstring: Pointer to the string where temperature data will be stored.
 * @param maxLen: Maximum length of the string.
 * @retval: None
 */
void SampleTempCToString(char *cstring,size_t maxLen){
	float temp;
	SampleTempCelsius(&temp); // Sample the temperature in Celsius
	snprintf(cstring,maxLen,"Temp(Celsius) | %0.2f\r\n",temp); // Convert to string
}

/***************************************************************************/
/*
 * @brief: Samples magnetometer data and converts it to a string.
 * @param cstring: Pointer to the string where magnetometer data will be stored.
 * @param maxLen: Maximum length of the string.
 * @retval: None
 */
void SampleMagMGaussToString(char *cstring,size_t maxLen){
	int x =0, y =0, z =0;
	SampleMagMGauss(&x,&y,&z); // Sample the magnetometer data
	snprintf(cstring,maxLen,"Mag(mGauss) | X: %d, Y: %d, Z: %d\r\n",x,y,z); // Convert to string
}

/***************************************************************************/
/*
 * @brief: Samples accelerometer data and converts it to a string.
 * @param cstring: Pointer to the string where accelerometer data will be stored.
 * @param maxLen: Maximum length of the string.
 * @retval: None
 */
void SampleAccGToString(char *cstring,size_t maxLen){
	float x =0, y =0, z =0;
	SampleAccG(&x,&y,&z); // Sample the accelerometer data
	snprintf(cstring,maxLen,"Acc(G) | X: %.2f, Y: %.2f, Z: %.2f\r\n",x,y,z); // Convert to string
}

/***************************************************************************/
/*
 * @brief: Samples gyroscope data and converts it to a string.
 * @param cstring: Pointer to the string where gyroscope data will be stored.
 * @param maxLen: Maximum length of the string.
 * @retval: None
 */
void SampleGyroDPSToString(char *cstring, size_t maxLen) {
    float x = 0, y = 0, z = 0;
    SampleGyroDPS(&x, &y, &z); // Sample the gyroscope data
    snprintf(cstring, maxLen, "Gyro(DPS) | X: %.2f, Y: %.2f, Z: %.2f\r\n", x, y, z); // Convert to string
}

/***************************************************************************/
/*
 * brief: Callback function triggered by a timer to manage data streaming.
 * param xTimerStream: Handle of the timer that triggered the callback.
 * retval: None
 */
void StreamTimeCallback(TimerHandle_t xTimerStream){
	/* Increment sample counter */
	++SampleCount;

	/* Stream mode to port: Send samples to port */
	if(STREAM_MODE_TO_PORT == StreamMode){
		if((SampleCount <= PortNumOfSamples) || (0 == PortNumOfSamples)){
			SampleToPort(PortModule,PortNumber,PortFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
	/* Stream mode to terminal: Export to terminal */
	else if(STREAM_MODE_TO_TERMINAL == StreamMode){
		if((SampleCount <= TerminalNumOfSamples) || (0 == TerminalNumOfSamples)){
			SampleToTerminal(TerminalPort,TerminalFunction);
		}
		else{
			SampleCount =0;
			xTimerStop(xTimerStream,0);
		}
	}
}

/***************************************************************************/
/*
 * brief: Streams a single sensor data sample to the terminal.
 * param dstPort: Port number to stream data to.
 * param dataFunction: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 * param numOfSamples: Number of samples (kept for compatibility, not used for repetition).
 * param streamTimeout: Timeout period for the operation (in milliseconds).
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status SampleToTerminal(uint8_t dstPort, All_Data dataFunction)
{
  Module_Status Status = H0BR4_OK;      /* Initialize operation status as success */
  int8_t* PcOutputString = NULL;        /* Pointer to CLI output buffer */
  uint32_t Period = 0u;                 /* Calculated period for the operation */
  char CString[100] = {0};              /* Buffer for formatted output string */
  float X = 0.0f, Y = 0.0f, Z = 0.0f;   /* Variables for accelerometer and gyroscope data */
  int Xm = 0, Ym = 0, Zm = 0;           /* Variables for magnetometer data */
  float TempCelsius = 0.0f;             /* Variable for temperature data */

  /* Process data based on the requested sensor function */
  switch (dataFunction)
  {
    case ACC:
      /* Get the CLI output buffer for writing */
      PcOutputString = FreeRTOS_CLIGetOutputBuffer();
      /* Sample accelerometer data in G units */
      if (SampleAccG(&X, &Y, &Z) != H0BR4_OK)
      {
        return H0BR4_ERROR;             /* Return error if sampling fails */
      }
      /* Format accelerometer data into a string */
      snprintf(CString, 50, "Acc(G) | X: %.2f, Y: %.2f, Z: %.2f\r\n", X, Y, Z);
      /* Send the formatted string to the specified port */
      writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
      break;

    case GYRO:
      /* Get the CLI output buffer for writing */
      PcOutputString = FreeRTOS_CLIGetOutputBuffer();
      /* Sample gyroscope data in degrees per second */
      if (SampleGyroDPS(&X, &Y, &Z) != H0BR4_OK)
      {
        return H0BR4_ERROR;             /* Return error if sampling fails */
      }
      /* Format gyroscope data into a string */
      snprintf(CString, 50, "Gyro(DPS) | X: %.2f, Y: %.2f, Z: %.2f\r\n", X, Y, Z);
      /* Send the formatted string to the specified port */
      writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
      break;

    case MAG:
      /* Get the CLI output buffer for writing */
      PcOutputString = FreeRTOS_CLIGetOutputBuffer();
      /* Sample magnetometer data in milliGauss */
      if (SampleMagMGauss(&Xm, &Ym, &Zm) != H0BR4_OK)
      {
        return H0BR4_ERROR;             /* Return error if sampling fails */
      }
      /* Format magnetometer data into a string */
      snprintf(CString, 50, "Mag(mGauss) | X: %d, Y: %d, Z: %d\r\n", Xm, Ym, Zm);
      /* Send the formatted string to the specified port */
      writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
      break;

    case TEMP:
      /* Get the CLI output buffer for writing */
      PcOutputString = FreeRTOS_CLIGetOutputBuffer();
      /* Sample temperature data in Celsius */
      if (SampleTempCelsius(&TempCelsius) != H0BR4_OK)
      {
        return H0BR4_ERROR;             /* Return error if sampling fails */
      }
      /* Format temperature data into a string */
      snprintf(CString, 50, "Temp(Celsius) | %0.2f\r\n", TempCelsius);
      /* Send the formatted string to the specified port */
      writePxMutex(dstPort, (char*)CString, strlen((char*)CString), cmd500ms, HAL_MAX_DELAY);
      break;

    default:
      /* Return error for invalid sensor function */
      return H0BR4_ERR_WrongParams;
  }

  /* Return final status indicating success or prior error */
  return Status;
}

/***************************************************************************/
/*
 * @brief: Polling and sleep function to safely manage CLI stream.
 * @param period: The period to sleep in milliseconds.
 * @param Numofsamples: The number of samples to take.
 * @retval: Module status indicating success or error.
 */
static Module_Status PollingSleepCLISafe(uint32_t period,long Numofsamples){
	const unsigned DELTA_SLEEP_MS =100; // milliseconds
	long numDeltaDelay =period / DELTA_SLEEP_MS;
	unsigned lastDelayMS =period % DELTA_SLEEP_MS;

	while(numDeltaDelay-- > 0){
		/* Delay for the specified period */
		vTaskDelay(pdMS_TO_TICKS(DELTA_SLEEP_MS));

		/* Look for ENTER key to stop the stream */
		for(uint8_t chr =1; chr < MSG_RX_BUF_SIZE; chr++){
			if(UARTRxBuf[PcPort - 1][chr] == '\r'){
				UARTRxBuf[PcPort - 1][chr] =0;
				StopeCliStreamFlag =1;
				return H0BR4_ERR_TERMINATED;
			}
		}

		/* Check if streaming should be stopped */
		if(stopStream)
			return H0BR4_ERR_TERMINATED;
	}

	/* Delay for the last remaining period */
	vTaskDelay(pdMS_TO_TICKS(lastDelayMS));

	return H0BR4_OK;
}

/***************************************************************************/
/*
 * @brief: Calculates the average of an array of readings.
 * @param1: Pointer to an array of float readings.
 * @param2: Number of readings in the array.
 * @retval: The average value of the readings.
 */
float calculate_average(float *readings, int num_readings) {
    float sum = 0; // Initialize sum to 0
    for (int i = 0; i < num_readings; i++) {
        sum += readings[i]; // Add each reading to the sum
    }
    return (float)(sum / num_readings); // Return the average value
}

/***************************************************************************/
/*
 * @brief: Calibrates the sensor by calculating offsets for X, Y, and Z axes.
 * @param1: Number of readings to take for calibration.
 * @param2: Pointer to store the X-axis offset.
 * @param3: Pointer to store the Y-axis offset.
 * @param4: Pointer to store the Z-axis offset.
 * @note: This function assumes the Z-axis is aligned with gravity (1G).
 *        It should be called to determine the optimal offset values.
 * @note: This function writes the offset values to the registers. Once you get the desired output,
 *        take the appropriate offset values for your sensor and write them to the registers using the set_offsets function.
 *        Then, stop calling this function.
 * @note: It is recommended to keep the number of samples between 50 and 200.
 * @retval: None
 */

void ACC_SetOffset(int num_readings, int16_t *X_offset, int16_t *Y_offset, int16_t *Z_offset) {
    float accx, accy, accz;
    float z_avg, y_avg, x_avg;
    float x_readings[num_readings], y_readings[num_readings], z_readings[num_readings];
    HAL_Delay(1000); // Initial delay before starting the calibration

    // Collect data
    for (int i = 0; i < num_readings; i++) {
        HAL_Delay(10); // Delay between each reading
        SampleAccG(&accx, &accy, &accz); // Sample accelerometer data
        x_readings[i] = accx;
        y_readings[i] = accy;
        z_readings[i] = accz;
    }

    // Calculate average values
    x_avg = calculate_average(x_readings, num_readings);
    y_avg = calculate_average(y_readings, num_readings);
    z_avg = calculate_average(z_readings, num_readings);

    // Calculate offsets
    int16_t x_offset = 0 - (x_avg * 1000);
    int16_t y_offset = 0 - (y_avg * 1000);
    int16_t z_offset = 1000 - (z_avg * 1000);

    *X_offset = x_offset;
    *Y_offset = y_offset;
    *Z_offset = -z_offset;

    // Apply offsets
    set_offsets(&dev_ctx, x_offset, y_offset, -z_offset);
}

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/*
 * @brief: Samples accelerometer data in G.
 * @param accX: Pointer to store X-axis accelerometer data.
 * @param accY: Pointer to store Y-axis accelerometer data.
 * @param accZ: Pointer to store Z-axis accelerometer data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleAccG(float *accX, float *accY, float *accZ) {
    Module_Status status = H0BR4_OK;
    if ((status = LSM6DS3TR_C_SampleAccG(accX, accY, accZ)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples gyroscope data in DPS.
 * @param gyroX: Pointer to store X-axis gyroscope data.
 * @param gyroY: Pointer to store Y-axis gyroscope data.
 * @param gyroZ: Pointer to store Z-axis gyroscope data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleGyroDPS(float *gyroX, float *gyroY, float *gyroZ) {
    Module_Status status = H0BR4_OK;
    if ((status = LSM6DS3TR_C_SampleGyroDPS(gyroX, gyroY, gyroZ)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples magnetometer data in milliGauss.
 * @param magX: Pointer to store X-axis magnetometer data.
 * @param magY: Pointer to store Y-axis magnetometer data.
 * @param magZ: Pointer to store Z-axis magnetometer data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleMagMGauss(int *magX, int *magY, int *magZ) {
    Module_Status status = H0BR4_OK;
    if ((LSM303SampleMagMGauss(magX, magY, magZ)) != LSM303AGR_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples temperature data in Celsius.
 * @param temp: Pointer to store temperature data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleTempCelsius(float *temp) {
    Module_Status status = H0BR4_OK;
    if ((LSM6DS3TR_C_SampleTempCelsius(temp)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples temperature data in Fahrenheit.
 * @param temp: Pointer to store temperature data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleTempFahrenheit(float *temp) {
    Module_Status status = H0BR4_OK;
    if ((LSM6DS3TR_C_SampleTempFahrenheit(temp)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples raw gyroscope data.
 * @param gyroX: Pointer to store X-axis raw gyroscope data.
 * @param gyroY: Pointer to store Y-axis raw gyroscope data.
 * @param gyroZ: Pointer to store Z-axis raw gyroscope data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
    Module_Status status = H0BR4_OK;
    if ((status = LSM6DS3TR_C_SampleGyroRaw(gyroX, gyroY, gyroZ)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples raw accelerometer data.
 * @param accX: Pointer to store X-axis raw accelerometer data.
 * @param accY: Pointer to store Y-axis raw accelerometer data.
 * @param accZ: Pointer to store Z-axis raw accelerometer data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ) {
    Module_Status status = H0BR4_OK;
    if ((status = LSM6DS3TR_C_SampleAccRaw(accX, accY, accZ)) != LSM6DS3TR_C_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * @brief: Samples raw magnetometer data.
 * @param magX: Pointer to store X-axis raw magnetometer data.
 * @param magY: Pointer to store Y-axis raw magnetometer data.
 * @param magZ: Pointer to store Z-axis raw magnetometer data.
 * @retval: Module status indicating success or error.
 */
Module_Status SampleMagRaw(int16_t *magX, int16_t *magY, int16_t *magZ) {
    Module_Status status = H0BR4_OK;
    if ((status = LSM303SampleMagRaw(magX, magY, magZ)) != LSM303AGR_OK)
        return status = H0BR4_ERROR;
    return status;
}

/***************************************************************************/
/*
 * brief: Samples data and exports it to a specified port.
 * param dstModule: The module number to export data from.
 * param dstPort: The port number to export data to.
 * param dataFunction: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status SampleToPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction){
	Module_Status Status =H0BR4_OK;
	static uint8_t Temp[12] ={0}; /* Buffer for data transmission */
	float AccX =0.0f, AccY =0.0f, AccZ =0.0f;
	float GyroX =0.0f, GyroY =0.0f, GyroZ =0.0f;
	int MagX =0, MagY =0, MagZ =0;
	float TempCelsius =0.0f;

	/* Check if the port and module ID are valid */
	if((dstPort == 0) && (dstModule == myID)){
		return H0BR4_ERR_WrongParams;
	}

	/* Sample and export data based on function type */
	switch(dataFunction){
		case ACC:
			if(SampleAccG(&AccX,&AccY,&AccZ) != H0BR4_OK){
				return H0BR4_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&AccX) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&AccX) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&AccX) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&AccX) >> 24);
				Temp[4] =(uint8_t )((*(uint32_t* )&AccY) >> 0);
				Temp[5] =(uint8_t )((*(uint32_t* )&AccY) >> 8);
				Temp[6] =(uint8_t )((*(uint32_t* )&AccY) >> 16);
				Temp[7] =(uint8_t )((*(uint32_t* )&AccY) >> 24);
				Temp[8] =(uint8_t )((*(uint32_t* )&AccZ) >> 0);
				Temp[9] =(uint8_t )((*(uint32_t* )&AccZ) >> 8);
				Temp[10] =(uint8_t )((*(uint32_t* )&AccZ) >> 16);
				Temp[11] =(uint8_t )((*(uint32_t* )&AccZ) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],12 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				messageParams[1] =(H0BR4_OK == Status) ?BOS_OK: BOS_ERROR;
				messageParams[0] =FMT_FLOAT;
				messageParams[2] =3;
				messageParams[3] =(uint8_t )((*(uint32_t* )&AccX) >> 0);
				messageParams[4] =(uint8_t )((*(uint32_t* )&AccX) >> 8);
				messageParams[5] =(uint8_t )((*(uint32_t* )&AccX) >> 16);
				messageParams[6] =(uint8_t )((*(uint32_t* )&AccX) >> 24);
				messageParams[7] =(uint8_t )((*(uint32_t* )&AccY) >> 0);
				messageParams[8] =(uint8_t )((*(uint32_t* )&AccY) >> 8);
				messageParams[9] =(uint8_t )((*(uint32_t* )&AccY) >> 16);
				messageParams[10] =(uint8_t )((*(uint32_t* )&AccY) >> 24);
				messageParams[11] =(uint8_t )((*(uint32_t* )&AccZ) >> 0);
				messageParams[12] =(uint8_t )((*(uint32_t* )&AccZ) >> 8);
				messageParams[13] =(uint8_t )((*(uint32_t* )&AccZ) >> 16);
				messageParams[14] =(uint8_t )((*(uint32_t* )&AccZ) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(float) * 3) + 3);
			}
			break;

		case GYRO:
			if(SampleGyroDPS(&GyroX,&GyroY,&GyroZ) != H0BR4_OK){
				return H0BR4_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&GyroX) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&GyroX) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&GyroX) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&GyroX) >> 24);
				Temp[4] =(uint8_t )((*(uint32_t* )&GyroY) >> 0);
				Temp[5] =(uint8_t )((*(uint32_t* )&GyroY) >> 8);
				Temp[6] =(uint8_t )((*(uint32_t* )&GyroY) >> 16);
				Temp[7] =(uint8_t )((*(uint32_t* )&GyroY) >> 24);
				Temp[8] =(uint8_t )((*(uint32_t* )&GyroZ) >> 0);
				Temp[9] =(uint8_t )((*(uint32_t* )&GyroZ) >> 8);
				Temp[10] =(uint8_t )((*(uint32_t* )&GyroZ) >> 16);
				Temp[11] =(uint8_t )((*(uint32_t* )&GyroZ) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],12 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				messageParams[1] =(H0BR4_OK == Status) ?BOS_OK: BOS_ERROR;
				messageParams[0] =FMT_FLOAT;
				messageParams[2] =3;
				messageParams[3] =(uint8_t )((*(uint32_t* )&GyroX) >> 0);
				messageParams[4] =(uint8_t )((*(uint32_t* )&GyroX) >> 8);
				messageParams[5] =(uint8_t )((*(uint32_t* )&GyroX) >> 16);
				messageParams[6] =(uint8_t )((*(uint32_t* )&GyroX) >> 24);
				messageParams[7] =(uint8_t )((*(uint32_t* )&GyroY) >> 0);
				messageParams[8] =(uint8_t )((*(uint32_t* )&GyroY) >> 8);
				messageParams[9] =(uint8_t )((*(uint32_t* )&GyroY) >> 16);
				messageParams[10] =(uint8_t )((*(uint32_t* )&GyroY) >> 24);
				messageParams[11] =(uint8_t )((*(uint32_t* )&GyroZ) >> 0);
				messageParams[12] =(uint8_t )((*(uint32_t* )&GyroZ) >> 8);
				messageParams[13] =(uint8_t )((*(uint32_t* )&GyroZ) >> 16);
				messageParams[14] =(uint8_t )((*(uint32_t* )&GyroZ) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(float) * 3) + 3);
			}
			break;

		case MAG:
			if(SampleMagMGauss(&MagX,&MagY,&MagZ) != H0BR4_OK){
				return H0BR4_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&MagX) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&MagX) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&MagX) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&MagX) >> 24);
				Temp[4] =(uint8_t )((*(uint32_t* )&MagY) >> 0);
				Temp[5] =(uint8_t )((*(uint32_t* )&MagY) >> 8);
				Temp[6] =(uint8_t )((*(uint32_t* )&MagY) >> 16);
				Temp[7] =(uint8_t )((*(uint32_t* )&MagY) >> 24);
				Temp[8] =(uint8_t )((*(uint32_t* )&MagZ) >> 0);
				Temp[9] =(uint8_t )((*(uint32_t* )&MagZ) >> 8);
				Temp[10] =(uint8_t )((*(uint32_t* )&MagZ) >> 16);
				Temp[11] =(uint8_t )((*(uint32_t* )&MagZ) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],12 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				messageParams[1] =(H0BR4_OK == Status) ?BOS_OK: BOS_ERROR;
				messageParams[0] =FMT_INT32;
				messageParams[2] =3;
				messageParams[3] =(uint8_t )((*(uint32_t* )&MagX) >> 0);
				messageParams[4] =(uint8_t )((*(uint32_t* )&MagX) >> 8);
				messageParams[5] =(uint8_t )((*(uint32_t* )&MagX) >> 16);
				messageParams[6] =(uint8_t )((*(uint32_t* )&MagX) >> 24);
				messageParams[7] =(uint8_t )((*(uint32_t* )&MagY) >> 0);
				messageParams[8] =(uint8_t )((*(uint32_t* )&MagY) >> 8);
				messageParams[9] =(uint8_t )((*(uint32_t* )&MagY) >> 16);
				messageParams[10] =(uint8_t )((*(uint32_t* )&MagY) >> 24);
				messageParams[11] =(uint8_t )((*(uint32_t* )&MagZ) >> 0);
				messageParams[12] =(uint8_t )((*(uint32_t* )&MagZ) >> 8);
				messageParams[13] =(uint8_t )((*(uint32_t* )&MagZ) >> 16);
				messageParams[14] =(uint8_t )((*(uint32_t* )&MagZ) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(int) * 3) + 3);
			}
			break;

		case TEMP:
			if(SampleTempCelsius(&TempCelsius) != H0BR4_OK){
				return H0BR4_ERROR;
			}

			if(dstModule == myID || dstModule == 0){
				/* LSB first */
				Temp[0] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 0);
				Temp[1] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 8);
				Temp[2] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 16);
				Temp[3] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 24);

				writePxITMutex(dstPort,(char* )&Temp[0],4 * sizeof(uint8_t),10);
			}
			else{
				/* LSB first */
				messageParams[1] =(H0BR4_OK == Status) ?BOS_OK: BOS_ERROR;
				messageParams[0] =FMT_FLOAT;
				messageParams[2] =1;
				messageParams[3] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 0);
				messageParams[4] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 8);
				messageParams[5] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 16);
				messageParams[6] =(uint8_t )((*(uint32_t* )&TempCelsius) >> 24);

				SendMessageToModule(dstModule,CODE_READ_RESPONSE,(sizeof(float) * 1) + 3);
			}
			break;

		default:
			return H0BR4_ERR_WrongParams;
	}

	/* Clear the temp buffer */
	memset(&Temp[0],0,sizeof(Temp));

	return Status;
}

/***************************************************************************/
/*
 * @brief: Streams data to a buffer.
 * @param buffer: Pointer to the buffer where data will be stored.
 * @param function: Function to sample data (e.g., ACC, GYRO, MAG, TEMP).
 * @param Numofsamples: Number of samples to take.
 * @param timeout: Timeout period for the operation.
 * @retval: Module status indicating success or error.
 */
Module_Status StreamToBuffer(float *buffer, All_Data function, uint32_t Numofsamples, uint32_t timeout) {
    switch (function) {
        case ACC:
            return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleAccBuf);
            break;
        case GYRO:
            return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleGyroBuf);
            break;
        case MAG:
            return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleMagBuf);
            break;
        case TEMP:
            return StreamMemsToBuf(buffer, Numofsamples, timeout, SampleTempBuff);
            break;
        default:
            break;
    }
}

/***************************************************************************/
/*
 * brief: Streams data to the specified port and module with a given number of samples.
 * param targetModule: The target module to which data will be streamed.
 * param portNumber: The port number on the module.
 * param portFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamtoPort(uint8_t dstModule,uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H0BR4_OK;
	uint32_t SamplePeriod =0u;

	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
		return H0BR4_ERROR; /* Assuming H0BR4_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_PORT;
	PortModule =dstModule;
	PortNumber =dstPort;
	PortFunction =dataFunction;
	PortNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H0BR4_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H0BR4_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H0BR4_ERROR;
	}

	return Status;
}

/***************************************************************************/
/*
 * brief: Streams data to the specified terminal port with a given number of samples.
 * param targetPort: The port number on the terminal.
 * param dataFunction: Type of data that will be streamed (ACC, GYRO, MAG, or TEMP).
 * param numOfSamples: The number of samples to stream.
 * param streamTimeout: The interval (in milliseconds) between successive data transmissions.
 * retval: of type Module_Status indicating the success or failure of the operation.
 */
Module_Status StreamToTerminal(uint8_t dstPort,All_Data dataFunction,uint32_t numOfSamples,uint32_t streamTimeout){
	Module_Status Status =H0BR4_OK;
	uint32_t SamplePeriod =0u;
	/* Check timer handle and timeout validity */
	if((NULL == xTimerStream) || (0 == streamTimeout) || (0 == numOfSamples)){
	return H0BR4_ERROR; /* Assuming H0BR4_ERROR is defined in Module_Status */
	}

	/* Set streaming parameters */
	StreamMode = STREAM_MODE_TO_TERMINAL;
	TerminalPort =dstPort;
	TerminalFunction =dataFunction;
	TerminalTimeout =streamTimeout;
	TerminalNumOfSamples =numOfSamples;

	/* Calculate the period from timeout and number of samples */
	SamplePeriod =streamTimeout / numOfSamples;

	/* Stop (Reset) the TimerStream if it's already running */
	if(xTimerIsTimerActive(xTimerStream)){
		if(pdFAIL == xTimerStop(xTimerStream,100)){
			return H0BR4_ERROR;
		}
	}

	/* Start the stream timer */
	if(pdFAIL == xTimerStart(xTimerStream,100)){
		return H0BR4_ERROR;
	}

	/* Update timer timeout - This also restarts the timer */
	if(pdFAIL == xTimerChangePeriod(xTimerStream,SamplePeriod,100)){
		return H0BR4_ERROR;
	}

	return Status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
static portBASE_TYPE SampleSensorCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const char *const AccCmdName ="acc";
	const char *const GyroCmdName ="gyro";
	const char *const MagCmdName ="mag";
	const char *const TempCmdName ="temp";

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen =0;

	// Make sure we return something
	*pcWriteBuffer ='\0';

	pSensName =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,1,&sensNameLen);

	if(pSensName == NULL){
		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		return pdFALSE;
	}

	do{
		if(!strncmp(pSensName,AccCmdName,strlen(AccCmdName))){
			SampleToTerminal(PcPort,ACC);


		}
		else if(!strncmp(pSensName,GyroCmdName,strlen(GyroCmdName))){
			SampleToTerminal(PcPort,GYRO);

		}
		else if(!strncmp(pSensName,MagCmdName,strlen(MagCmdName))){
			SampleToTerminal(PcPort,MAG);

		}
		else if(!strncmp(pSensName,TempCmdName,strlen(TempCmdName))){
			SampleToTerminal(PcPort,TEMP);

		}
		else{
			snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		}

		return pdFALSE;
	} while(0);

	snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Error reading Sensor\r\n");
	return pdFALSE;
}

/***************************************************************************/
// Port Mode => false and CLI Mode => true
static bool StreamCommandParser(const int8_t *pcCommandString,const char **ppSensName,portBASE_TYPE *pSensNameLen,
bool *pPortOrCLI,uint32_t *pPeriod,uint32_t *pTimeout,uint8_t *pPort,uint8_t *pModule){
	const char *pPeriodMSStr = NULL;
	const char *pTimeoutMSStr = NULL;

	portBASE_TYPE periodStrLen =0;
	portBASE_TYPE timeoutStrLen =0;

	const char *pPortStr = NULL;
	const char *pModStr = NULL;

	portBASE_TYPE portStrLen =0;
	portBASE_TYPE modStrLen =0;

	*ppSensName =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,1,pSensNameLen);
	pPeriodMSStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,2,&periodStrLen);
	pTimeoutMSStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,3,&timeoutStrLen);

	// At least 3 Parameters are required!
	if((*ppSensName == NULL) || (pPeriodMSStr == NULL) || (pTimeoutMSStr == NULL))
		return false;

	// TODO: Check if Period and Timeout are integers or not!
	*pPeriod =atoi(pPeriodMSStr);
	*pTimeout =atoi(pTimeoutMSStr);
	*pPortOrCLI = true;

	pPortStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,4,&portStrLen);
	pModStr =(const char* )FreeRTOS_CLIGetParameter(pcCommandString,5,&modStrLen);

	if((pModStr == NULL) && (pPortStr == NULL))
		return true;
	if((pModStr == NULL) || (pPortStr == NULL))	// If user has provided 4 Arguments.
		return false;

	*pPort =atoi(pPortStr);
	*pModule =atoi(pModStr);
	*pPortOrCLI = false;

	return true;
}

/***************************************************************************/
static portBASE_TYPE StreamSensorCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString){
	const char *const AccCmdName ="acc";
	const char *const GyroCmdName ="gyro";
	const char *const MagCmdName ="mag";
	const char *const TempCmdName ="temp";

	uint32_t Numofsamples =0;
	uint32_t timeout =0;
	uint8_t port =0;
	uint8_t module =0;

	bool portOrCLI = true; // Port Mode => false and CLI Mode => true

	const char *pSensName = NULL;
	portBASE_TYPE sensNameLen =0;

	// Make sure we return something
	*pcWriteBuffer ='\0';

	if(!StreamCommandParser(pcCommandString,&pSensName,&sensNameLen,&portOrCLI,&Numofsamples,&timeout,&port,&module)){
		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		return pdFALSE;
	}

	do{
		if(!strncmp(pSensName,AccCmdName,strlen(AccCmdName))){
			if(portOrCLI){
				StreamToCLI(Numofsamples,timeout,SampleAccGToString);
			}

		}
		else if(!strncmp(pSensName,GyroCmdName,strlen(GyroCmdName))){
			if(portOrCLI){
				StreamToCLI(Numofsamples,timeout,SampleGyroDPSToString);

			}
		}
		else if(!strncmp(pSensName,MagCmdName,strlen(MagCmdName))){
			if(portOrCLI){
				StreamToCLI(Numofsamples,timeout,SampleMagMGaussToString);

			}

		}
		else if(!strncmp(pSensName,TempCmdName,strlen(TempCmdName))){
			if(portOrCLI){
				StreamToCLI(Numofsamples,timeout,SampleTempCToString);

			}

		}
		else{
			snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Invalid Arguments\r\n");
		}

		snprintf((char* )pcWriteBuffer,xWriteBufferLen,"\r\n");
		return pdFALSE;
	} while(0);

	snprintf((char* )pcWriteBuffer,xWriteBufferLen,"Error reading Sensor\r\n");
	return pdFALSE;
}
/***************************************************************************/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
