#include "main.h"
#include "UART_LL.h"

volatile uint16_t ReceiveBufferPosition = 0;
volatile uint8_t receiveBuffer[REC_BUFFER_SIZE] = {0x00};
volatile uint8_t errorCounter = 0;

DMA_UART_TypeDef DMAUart_Struct = {
	.sendReceiveBuffer = {0x00},
	.bufferPosition = 0,
	.receiveDMA_ControlFlow_Flag = 0,
	.dmaReceiveData = 0,
	.transmitComplete_Flag = 0
};

//---------------------------------DMA Functions------------------------------------
void Usart_DMA_Transmit(uint16_t sendDataLength)
{
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, sendDataLength);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

void USART_Transmit_WaitUntillFinished(uint16_t transmitDataLength)
{
  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, transmitDataLength);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
  while (!DMAUart_Struct.transmitComplete_Flag) {}
  DMAUart_Struct.transmitComplete_Flag=0;
}

void Usart_DMA_Receive(uint16_t recDataLength)
{
	if(DMAUart_Struct.receiveDMA_ControlFlow_Flag == 2)
	{
		DMAUart_Struct.receiveDMA_ControlFlow_Flag=0;
		Usart_DMA_Transmit(recDataLength);
	}

	if(DMAUart_Struct.receiveDMA_ControlFlow_Flag == 0)
	{
		UART_DMA_EnableReceive(recDataLength);
	}
}

void UART_DMA_EnableReceive(uint16_t dataLength)
{
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, dataLength);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
	DMAUart_Struct.receiveDMA_ControlFlow_Flag=1;
}

void DMA2_RecieveComplete(void)
{
	DMAUart_Struct.receiveDMA_ControlFlow_Flag = 2;

	DMAUart_Struct.sendReceiveBuffer[DMAUart_Struct.bufferPosition] = (uint8_t)DMAUart_Struct.dmaReceiveData;
	DMAUart_Struct.bufferPosition++;

    if(DMAUart_Struct.bufferPosition == 200)
    {
    	DMAUart_Struct.bufferPosition = 0;
    }

    Usart_DMA_Receive(1);
}

void DMA2_TransmitComplete(void)
{
	DMAUart_Struct.transmitComplete_Flag = 1;
}
//---------------------------------------------------------------------------------------
void UsartSendData(USART_TypeDef* UART_PORT, uint8_t* framePtr, uint16_t frameSize)
{
  uint16_t dataCounter = 0;

  while (dataCounter<frameSize)
  {
    while (!CHECK_IF_DATA_SEND_UART_FINISHED(UART_PORT)) { }
    LL_USART_TransmitData8(UART_PORT,*(uint8_t*)(framePtr + (dataCounter++)));
  }
}

/*
 * Function Blocking whole project.
 * It waits till it receive data
 */
uint8_t UartRecSingleChar(USART_TypeDef* UART_PORT)
{
  uint8_t recData = 0;

  while (!CHECK_IF_DATA_RECEIVE(USART1)) {}
  recData = (uint8_t)(USART1->DR & 0x00FF);

  return recData;
}

uint16_t UartRecWholeBuffer(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize)
{
	uint16_t dataLoop = 0;
	uint16_t receiveDataCounter = 0;

	while(1)
	{
		volatile uint32_t timeout = 0;

		while (!CHECK_IF_DATA_RECEIVE(UART_PORT)) {
			timeout++;
			if(timeout == REC_TIMEOUT) { break; }
		}

		if(timeout == REC_TIMEOUT) { break; }
		else
		{
			*(framePtr + dataLoop) = (uint8_t)(UART_PORT->DR & 0x00FF);
			dataLoop++;
			receiveDataCounter++;

			if(receiveDataCounter == frameSize)
			{
				dataLoop = 0;
			}
		}
	}

	return receiveDataCounter;
}

uint16_t UartRecWholeBufferWithSpecValueAtBufEnd(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize)
{
	uint16_t dataLoop = 0;
	uint16_t receiveDataCounter = 0;

	while(1)
	{
		volatile uint32_t timeout = 0;

		while (!CHECK_IF_DATA_RECEIVE(UART_PORT)) {
			timeout++;
			if(timeout == REC_TIMEOUT) { break; }
		}

		if(timeout == REC_TIMEOUT) { break; }
		else
		{
			*(framePtr + dataLoop) = (uint8_t)(UART_PORT->DR & 0x00FF);

            if(*(framePtr + dataLoop) == 0x00)
            {
                break;
            }

			dataLoop++;
			receiveDataCounter++;

			if(receiveDataCounter == frameSize)
			{
				dataLoop = 0;
			}
		}
	}

	return receiveDataCounter;
}

uint16_t UartRecWholeBufferWithSpecFrameSize(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize)
{
	uint16_t dataLoop = 0;
	uint16_t receiveDataCounter = 0;
	uint16_t expectedFrameSize = 0;

	while(1)
	{
		volatile uint32_t timeout = 0;

		while (!CHECK_IF_DATA_RECEIVE(UART_PORT)) {
			timeout++;
			if(timeout == REC_TIMEOUT) { break; }
		}

		if(timeout == REC_TIMEOUT) { break; }
		else
		{
			*(framePtr + dataLoop) = (uint8_t)(UART_PORT->DR & 0x00FF);

            if((dataLoop) == 1)
            {
            	expectedFrameSize = *(framePtr + dataLoop);
            }

			dataLoop++;
			receiveDataCounter++;

			if(receiveDataCounter == frameSize)
			{
				dataLoop = 0;
			}
			else if(expectedFrameSize == receiveDataCounter)
			{
				break;
			}
		}
	}

	return receiveDataCounter;
}

/*
 * Use after initialization of UART
 * */
void UartEnableInterrupt(USART_TypeDef* UART_PORT)
{
	LL_USART_EnableIT_RXNE(UART_PORT);
	LL_USART_EnableIT_ERROR(UART_PORT);
}

static void USART_RX_Receive(USART_TypeDef* UART_PORT, uint8_t *rxBufPtr)
{
	*(rxBufPtr + ReceiveBufferPosition) = LL_USART_ReceiveData8(UART_PORT);
	ReceiveBufferPosition++;

	if(ReceiveBufferPosition == (REC_BUFFER_SIZE-1))
	{
		ReceiveBufferPosition=0;
	}
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  {
	  USART_RX_Receive(USART1, &receiveBuffer[0]);
  }
  else
  {
	/* Check if ERROR flag occure */
    if(LL_USART_IsActiveFlag_ORE(USART1)) {
    	errorCounter++;
    }
    else if(LL_USART_IsActiveFlag_FE(USART1)) {
    	errorCounter++;
    }
    else if(LL_USART_IsActiveFlag_NE(USART1)) {
    	errorCounter++;
    }
  }
}
