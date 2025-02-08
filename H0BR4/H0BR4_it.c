/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H01R0_it.c
 Description   :Interrupt Service Routines.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

uint8_t temp_length[NumOfPorts] = {0};
uint8_t temp_index[NumOfPorts] = {0};
uint8_t* error_restart_message = "Restarting...\r\n";


/* External variables --------------------------------------------------------*/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
extern uint8_t UARTRxBufIndex[NumOfPorts];
extern uint8_t WakeupFromStopFlag;

/* External function prototypes ----------------------------------------------*/
extern TaskHandle_t xCommandConsoleTaskHandle; // CLI Task handler.

uint16_t PacketLength = 0;
uint8_t count = 0;
/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void){
	
	HAL_IncTick();
	osSystickHandler();
	
}

/**
 * @brief This function handles Hard Fault error callback.
 */
void HardFault_Handler(void){
	/* Loop here */
	uint8_t* error_message = "HardFault Error\r\n";
	writePxMutex(PcPort, (char*) error_message, 17, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
	for(;;){
	};
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* Use of HAL_UARTEx_ReceiveToIdle_DMA service, will generate calls to
   user defined HAL_UARTEx_RxEventCallback callback for each occurrence of
    following events :
    - HT (Half Transfer) : Half of Rx buffer is filled)
    - TC (Transfer Complete) : Rx buffer is full.
      (In case of Circular DMA, reception could go on, and next reception data will be stored
      in index 0 of reception buffer by DMA).
    - Idle Event on Rx line : Triggered when RX line has been in idle state (normally high state)
      for 1 frame time, after last received byte. */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size){
	extern TaskHandle_t BackEndTaskHandle;

	PacketLength =Size;
	count++;

	/* Notify backend task */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(BackEndTaskHandle,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart1)		
	HAL_UART_IRQHandler(&huart1);
#endif

	/* Fix problem stuck CPU in UART IRQhandler because of error on UART bus through use
	 * HAL_UART_Transmit_IT() ,this prevented the TXFNFIE flag from being cleared which caused this problem
	 */
	if( (READ_BIT(huart1.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart1.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart1.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart1.Instance->CR1, USART_CR1_TCIE);
	}
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
 */
void USART2_LPUART2_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart2)	
	HAL_UART_IRQHandler(&huart2);
#endif
	
	if( (READ_BIT(huart2.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart2.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart2.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart2.Instance->CR1, USART_CR1_TCIE);
	}
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
 */

void USART3_4_5_6_LPUART1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart3)
	HAL_UART_IRQHandler(&huart3);
#endif
#if defined (_Usart4)
	HAL_UART_IRQHandler(&huart4);
#endif
#if defined (_Usart5)
	HAL_UART_IRQHandler(&huart5);
#endif
#if defined (_Usart6)
	HAL_UART_IRQHandler(&huart6);
#endif

	if( (READ_BIT(huart3.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart3.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart3.Instance->CR1, USART_CR1_TCIE);
	}

	else if( (READ_BIT(huart4.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart4.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart4.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart4.Instance->CR1, USART_CR1_TCIE);
	}

	else if( (READ_BIT(huart5.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart5.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart5.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart5.Instance->CR1, USART_CR1_TCIE);
	}

	else if( (READ_BIT(huart6.Instance->CR1, USART_CR1_TXEIE_TXFNFIE) == USART_CR1_TXEIE_TXFNFIE_Msk) &&
			(huart6.gState == HAL_UART_STATE_READY))
	{
	      /* Disable the UART Transmit Data Register Empty Interrupt */
	      ATOMIC_CLEAR_BIT(huart6.Instance->CR1, USART_CR1_TXEIE_TXFNFIE);

	      /* Enable the UART Transmit Complete Interrupt */
	      ATOMIC_SET_BIT(huart6.Instance->CR1, USART_CR1_TCIE);
	}

	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 1 interrupt (Uplink DMA 1).
 */
void DMA1_Ch1_IRQHandler(void){
	/* Streaming or messaging DMA on P1 */
//	DMA_IRQHandler(P1);

}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 2 to 3 and DMA2 channel 1 to 2 interrupts.
 */
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void){
//	/* Streaming or messaging DMA on P5 */
//	if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF2) == SET){
//		DMA_IRQHandler(P5);
//		/* Streaming or messaging DMA on P2 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF3) == SET){
//		DMA_IRQHandler(P2);
//		/* TX messaging DMA 0 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF2) == SET){
//		HAL_DMA_IRQHandler(&msgTxDMA[0]);
//	}
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 4 to 7 and DMA2 channel 3 to 5 interrupts.
 */
void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void){
//	/* Streaming or messaging DMA on P3 */
//	if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF5) == SET){
//		DMA_IRQHandler(P3);
//		/* Streaming or messaging DMA on P4 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF6) == SET){
//		DMA_IRQHandler(P4);
//		/* Streaming or messaging DMA on P6 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF3) == SET){
//		DMA_IRQHandler(P6);
//		/* TX messaging DMA 1 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF4) == SET){
//		HAL_DMA_IRQHandler(&msgTxDMA[1]);
//		/* TX messaging DMA 2 */
//	}
//	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF7) == SET){
//		HAL_DMA_IRQHandler(&msgTxDMA[2]);
//	}
}

/*-----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/* TX DMAs are shared so unsetup them here to be reused */
//	if(huart->hdmatx != NULL)
//		DMA_MSG_TX_UnSetup(huart);
	
	/* Give back the mutex. */
	xSemaphoreGiveFromISR(PxTxSemaphoreHandle[GetPort(huart)],&(xHigherPriorityTaskWoken));
}

/*-----------------------------------------------------------*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	/* Loop here */
	//for(;;) {};
	/* Set the UART state ready to be able to start the process again */
	huart->gState =HAL_UART_STATE_READY;
	
	/* Resume streaming DMA for this UART port */
	uint8_t port =GetPort(huart);
	if(portStatus[port] == STREAM){
//		HAL_UART_Receive_DMA(huart,(uint8_t* )(&(dmaStreamDst[port - 1]->Instance->TDR)),huart->hdmarx->Instance->CNDTR);
		HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t* )(&(dmaStreamDst[port - 1]->Instance->TDR)),huart->hdmarx->Instance->CNDTR);
		/* Or parse the circular buffer and restart messaging DMA for this port */
	}
	else{
		index_input[port - 1] = 0;
		index_process[port - 1] = 0;
		memset((uint8_t* )&UARTRxBuf[port - 1], 0, MSG_RX_BUF_SIZE);
//		HAL_UART_Receive_DMA(huart,(uint8_t* )&UARTRxBuf[port - 1] ,MSG_RX_BUF_SIZE);
		HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t* )&UARTRxBuf[port - 1] ,MSG_RX_BUF_SIZE);
		MsgDMAStopped[port - 1] = true;		// Set a flag here and let the backend task restart DMA after parsing the buffer	
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	uint8_t port_number = GetPort(huart);
//	uint8_t port_index = port_number - 1;
//	if(Rx_Data[port_index] == 0x0D && portStatus[port_number] == FREE)
//	{
//		for(int i=0;i<=NumOfPorts;i++) // Free previous CLI port
//		{
//			if(portStatus[i] == CLI)
//			{
//				portStatus[i] = FREE;
//			}
//		}
//		portStatus[port_number] =CLI; // Continue the CLI session on this port
//		PcPort = port_number;
//		xTaskNotifyGive(xCommandConsoleTaskHandle);
//
//		if(Activate_CLI_For_First_Time_Flag == 1) Read_In_CLI_Task_Flag = 1;
//		Activate_CLI_For_First_Time_Flag = 1;
//
//	}
//	else if(portStatus[port_number] == CLI)
//	{
//		Read_In_CLI_Task_Flag = 1;
//	}
//
//	else if(Rx_Data[port_index] == 'H' && portStatus[port_number] == FREE)
//	{
//		portStatus[port_number] =H_Status; // H  Character was received, waiting for Z character.
//	}
//
//	else if(Rx_Data[port_index] == 'Z' && portStatus[port_number] == H_Status)
//	{
//		portStatus[port_number] =Z_Status; // Z  Character was received, waiting for length byte.
//	}
//
//	else if(Rx_Data[port_index] != 'Z' && portStatus[port_number] == H_Status)
//	{
//		portStatus[port_number] =FREE; // Z  Character was not received, so there is no message to receive.
//	}
//
//	else if(portStatus[port_number] == Z_Status)
//	{
//		portStatus[port_number] =MSG; // Receive length byte.
//		MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][2] = Rx_Data[port_index];
//		temp_index[port_index] = 3;
//		temp_length[port_index] = Rx_Data[port_index] + 1;
//	}
//
//	else if(portStatus[port_number] == MSG)
//	{
//		if(temp_length[port_index] > 1)
//		{
//			MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] = Rx_Data[port_index];
//			temp_index[port_index]++;
//			temp_length[port_index]--;
//		}
//		else
//		{
//			MSG_Buffer[port_index][MSG_Buffer_Index_End[port_index]][temp_index[port_index]] = Rx_Data[port_index];
//			temp_index[port_index]++;
//			temp_length[port_index]--;
//			MSG_Buffer_Index_End[port_index]++;
//			if(MSG_Buffer_Index_End[port_index] == MSG_COUNT) MSG_Buffer_Index_End[port_index] = 0;
//
//
//			Process_Message_Buffer[Process_Message_Buffer_Index_End] = port_number;
//			Process_Message_Buffer_Index_End++;
//			if(Process_Message_Buffer_Index_End == MSG_COUNT) Process_Message_Buffer_Index_End = 0;
//			portStatus[port_number] =FREE; // End of receiving message.
//		}
//	}
//
////		HAL_UART_Receive_DMA(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
//	HAL_UART_Receive_IT(huart,(uint8_t* )&Rx_Data[GetPort(huart) - 1] , 1);
}

/*-----------------------------------------------------------*/

/**
  * @brief UART wakeup from Stop mode callback
  * @param huart: uart handle
  * @retval None
  */
void HAL_UARTEx_WakeupCallback(UART_HandleTypeDef *huart) {

	WakeupFromStopFlag = 1;

	if (huart->Instance == USART1)
		HAL_UARTEx_DisableStopMode(huart);

	if (huart->Instance == USART2)
		HAL_UARTEx_DisableStopMode(huart);

	if (huart->Instance == USART3)
		HAL_UARTEx_DisableStopMode(huart);

}

/*-----------------------------------------------------------*/

/* Run time stack overflow checking is performed if
 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
 function is called if a stack overflow is detected. */
void vApplicationStackOverflowHook( xTaskHandle pxTask,signed char *pcTaskName){
	(void )pcTaskName;
	(void )pxTask;
	uint8_t* error_message = "Stack Overflow\r\n";
	writePxMutex(PcPort, (char*) error_message, 16, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
//	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

/* vApplicationMallocFailedHook() will only be called if
 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
 function that will get called if a call to pvPortMalloc() fails.
 pvPortMalloc() is called internally by the kernel whenever a task, queue,
 timer or semaphore is created.  It is also called by various parts of the
 demo application.  If heap_1.c or heap_2.c are used, then the size of the
 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
 to query the size of free heap space that remains (although it does not
 provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void){
	uint8_t* error_message = "Heap size exceeded\r\n";
	writePxMutex(PcPort, (char*) error_message, 20, 0xff, 0xff);
	writePxMutex(PcPort, (char*) error_restart_message, 15, 0xff, 0xff);
	NVIC_SystemReset();
//	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
