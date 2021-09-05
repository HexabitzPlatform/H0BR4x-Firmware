/*
 BitzOS (BOS) V0.2.5 - Copyright (C) 2017-2021 Hexabitz
 All rights reserved

 File Name     : H0BR4_it.c
 Description   :Interrupt Service Routines.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* External variables --------------------------------------------------------*/
extern uint8_t UARTRxBuf[NumOfPorts][MSG_RX_BUF_SIZE];
//extern uint8_t UARTTxBuf[3][MSG_TX_BUF_SIZE];
extern uint8_t UARTRxBufIndex[NumOfPorts];

/* External function prototypes ----------------------------------------------*/

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
	for(;;){
	};
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
 */
void USART1_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart1)		
	HAL_UART_IRQHandler(&huart1);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
 */
void USART2_IRQHandler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
#if defined (_Usart2)	
	HAL_UART_IRQHandler(&huart2);
#endif
	
	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	 switch should be performed before the interrupt exists.  That ensures the
	 unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
 */
void USART3_8_IRQHandler(void){
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
	DMA_IRQHandler(P1);
	
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 2 to 3 and DMA2 channel 1 to 2 interrupts.
 */
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void){
	/* Streaming or messaging DMA on P5 */
	if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF2) == SET){
		DMA_IRQHandler(P5);
		/* Streaming or messaging DMA on P2 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF3) == SET){
		DMA_IRQHandler(P2);
		/* TX messaging DMA 0 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF2) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[0]);
	}
}

/*-----------------------------------------------------------*/

/**
 * @brief This function handles DMA1 channel 4 to 7 and DMA2 channel 3 to 5 interrupts.
 */
void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void){
	/* Streaming or messaging DMA on P3 */
	if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF5) == SET){
		DMA_IRQHandler(P3);
		/* Streaming or messaging DMA on P4 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF6) == SET){
		DMA_IRQHandler(P4);
		/* Streaming or messaging DMA on P6 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA2,DMA_ISR_GIF3) == SET){
		DMA_IRQHandler(P6);
		/* TX messaging DMA 1 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF4) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[1]);
		/* TX messaging DMA 2 */
	}
	else if(HAL_DMA_GET_IT_SOURCE(DMA1,DMA_ISR_GIF7) == SET){
		HAL_DMA_IRQHandler(&msgTxDMA[2]);
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	/* TX DMAs are shared so unsetup them here to be reused */
	if(huart->hdmatx != NULL)
		DMA_MSG_TX_UnSetup(huart);
	
	/* Give back the mutex. */
	xSemaphoreGiveFromISR(PxTxSemaphoreHandle[GetPort(huart)],&(xHigherPriorityTaskWoken));
}

/*-----------------------------------------------------------*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	/* Loop here */
	//for(;;) {};
	/* Set the UART state ready to be able to start the process again */
	huart->State =HAL_UART_STATE_READY;
	
	/* Resume streaming DMA for this UART port */
	uint8_t port =GetPort(huart);
	if(portStatus[port] == STREAM){
		HAL_UART_Receive_DMA(huart,(uint8_t* )(&(dmaStreamDst[port - 1]->Instance->TDR)),huart->hdmarx->Instance->CNDTR);
		/* Or parse the circular buffer and restart messaging DMA for this port */
	}
	else{
		MsgDMAStopped[port - 1] = true;		// Set a flag here and let the backend task restart DMA after parsing the buffer	
	}
}

/*-----------------------------------------------------------*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// Check only ports in messaging mode
	if(portStatus[GetPort(huart)] == FREE || portStatus[GetPort(huart)] == MSG){
		// Circular buffer is full. Set a global persistant flag via BOS events and a temporary flag via portStatus.
		BOSMessaging.overrun =GetPort(huart);
		portStatus[GetPort(huart)] =OVERRUN;
		// Reset the circular RX buffer index
		UARTRxBufIndex[GetPort(huart) - 1] =0;
		// Set a port-specific flag here and let the backend task restart DMA
		MsgDMAStopped[GetPort(huart) - 1] = true;
	}
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask,signed char *pcTaskName){
	(void )pcTaskName;
	(void )pxTask;
	
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void){
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
	taskDISABLE_INTERRUPTS();
	for(;;);
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
