#ifndef UART_LL_C_
#define UART_LL_C_

#define REC_TIMEOUT 0x00000FFF
#define REC_BUFFER_SIZE 1024
#define CHECK_IF_DATA_SEND_UART_FINISHED(UART) LL_USART_IsActiveFlag_TXE(UART)
#define CHECK_IF_DATA_RECEIVE(UART) 	LL_USART_IsActiveFlag_RXNE(UART)

typedef struct DMA_UART_SendReceiveData{
	uint8_t sendReceiveBuffer[200];
	uint8_t bufferPosition;
	volatile uint8_t receiveDMA_ControlFlow_Flag;
	uint32_t dmaReceiveData;
	volatile uint8_t transmitComplete_Flag;
}DMA_UART_TypeDef;

extern DMA_UART_TypeDef DMAUart_Struct;

extern volatile uint8_t receiveBuffer[1024];
extern volatile uint16_t ReceiveBufferPosition;

void UsartSendData(USART_TypeDef* UART_PORT, uint8_t* framePtr, uint16_t frameSize);

uint8_t UartRecSingleChar(USART_TypeDef* UART_PORT);
uint16_t UartRecWholeBuffer(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize);
uint16_t UartRecWholeBufferWithSpecValueAtBufEnd(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize);
uint16_t UartRecWholeBufferWithSpecFrameSize(USART_TypeDef* UART_PORT, uint8_t* framePtr,
							uint16_t frameSize);
void UartEnableInterrupt(USART_TypeDef* UART_PORT);
void USART_Transmit_WaitUntillFinished(uint16_t transmitDataLength);
//---------------------------------------------------
void Usart_DMA_Transmit(uint16_t sendDataLength);
void DMA2_RecieveComplete(void);
void Usart_DMA_Receive(uint16_t recDataLength);
void DMA2_TransmitComplete(void);
void UART_DMA_EnableReceive(uint16_t dataLength);

#endif /* UART_LL_C_ */
