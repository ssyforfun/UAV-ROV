// 初始化 串口
#pragma region 自定义 part 1 
#include "uart_computer.h"
#pragma endregion

#pragma region common part 1
static UART_HandleTypeDef UartHandle;
static DMA_HandleTypeDef UartDmaTxHandle, UartDmaRxHandle;
static void uart_default_init(void);
#pragma endregion

// --------------- 需要自定义的部分 --------------------------------
#pragma region 自定义 part 2
UART_HandleTypeDef *huart5Handle = &UartHandle;
void (*uart5_init)(void) = uart_default_init;
#define BAUD_RATE (115200) // 波特率, 常见有: 9600, 115200, 38400
#define USE_UART5 1
#define USART_INSTANCE UART5
#define USART_INSTANCE_CLOCK_ENABLE() __HAL_RCC_UART5_CLK_ENABLE()
#define USART_INSTANCE_DMA_CLOCK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
#define USART_INSTANCE_TX_DMA_CHANNEL DMA_CHANNEL_4 // refer to reference manual page 307 - 308
#define USART_INSTANCE_RX_DMA_CHANNEL DMA_CHANNEL_4
#define USART_INSTANCE_TX_DMA_STREAM DMA1_Stream7
#define USART_INSTANCE_RX_DMA_STREAM DMA1_Stream0
#pragma endregion

#pragma region common part 2
#define RXBUFFERSIZE PROTOCOL_BUFFER_SIZE
#define TXBUFFERSIZE PROTOCOL_BUFFER_SIZE
static void UART_PP_Init(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t stopbit, uint32_t parity, uint32_t bufsize);
static void uart_dma_init(UART_HandleTypeDef *huart);
static uint8_t TxMemory[RXBUFFERSIZE]; // for DMA Tx : user buffer -> frame buffer -> fifo buffer -> DMA buffer(TxMemory)
static uint8_t RxMemory[RXBUFFERSIZE]; // for DMA Rx
#pragma endregion

#pragma region common part 3
// --------------- 基本固定部分 ------------------------------------
static void uart_default_init(void)
{
    USART_INSTANCE_CLOCK_ENABLE();
    USART_INSTANCE_DMA_CLOCK_ENABLE();                            // 打开串口GPIO时钟
    UartHandle.Instance = USART_INSTANCE;                         // USART1
    UartHandle.Instance = USART_INSTANCE;                         // USART3
    UartDmaTxHandle.Instance = USART_INSTANCE_TX_DMA_STREAM;      // DMA1_Stream3
    UartDmaTxHandle.Init.Channel = USART_INSTANCE_TX_DMA_CHANNEL; // DMA_CHANNEL_4
    UartDmaRxHandle.Instance = USART_INSTANCE_RX_DMA_STREAM;      // DMA1_Stream1
    UartDmaRxHandle.Init.Channel = USART_INSTANCE_RX_DMA_CHANNEL; // DMA_CHANNEL_4
    UartHandle.pTxBuffPtr = TxMemory;
    UartHandle.pRxBuffPtr = RxMemory;
    __HAL_LINKDMA(&UartHandle, hdmatx, UartDmaTxHandle);
    __HAL_LINKDMA(&UartHandle, hdmarx, UartDmaRxHandle);
    UART_PP_Init(&UartHandle, BAUD_RATE, 1, 0, RXBUFFERSIZE);
}

// ---------- 常用通用串口配置函数 -----------------------------------------------------------------------------------------
static void UART_PP_Init(UART_HandleTypeDef *huart, uint32_t baudrate, uint32_t stopbit, uint32_t parity, uint32_t bufsize)
{
    // ##-1- Configure the UART peripheral ######################################
    // Put the USART peripheral in the Asynchronous mode (UART Mode)
    // UART1 configured as follow:
    //    - Word Length = 8 Bits
    //    - Stop Bit    2->2 bit, others->1 bit
    //    - Parity      0->none, 1->odd, 2->even
    //    - BaudRate
    //    - Hardware flow control disabled (RTS and CTS signals)
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    if (stopbit == 2)
        huart->Init.StopBits = UART_STOPBITS_2;
    else
        huart->Init.StopBits = UART_STOPBITS_1;
    if (parity == 0)
        huart->Init.Parity = UART_PARITY_NONE;
    else if (parity == 1)
        huart->Init.Parity = UART_PARITY_ODD;
    else
        huart->Init.Parity = UART_PARITY_EVEN;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(huart);

    uart_dma_init(huart);

    HAL_UART_Receive_DMA(huart, huart->pRxBuffPtr, bufsize);
}
static void uart_dma_init(UART_HandleTypeDef *huart)
{
    DMA_HandleTypeDef *hdma_tx;
    DMA_HandleTypeDef *hdma_rx;

    hdma_tx = huart->hdmatx;
    hdma_rx = huart->hdmarx;

    // ##-3- Configure the DMA streams ##########################################
    // Configure the DMA handler for Transmission process
    // hdma_tx->Instance                 = DMA1_Channel4; // refer to reference manual page 299
    // hdma_tx->Init.Request             = DMA_REQUEST_2;
    hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_tx->Init.Mode = DMA_NORMAL;
    hdma_tx->Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_DeInit(hdma_tx);
    HAL_DMA_Init(hdma_tx);

    // Configure the DMA handler for reception process
    // hdma_rx->Instance                 = DMA1_Channel5; // refer to reference manual page 299
    // hdma_rx->Init.Request             = DMA_REQUEST_2;
    hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
    hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_rx->Init.Mode = DMA_CIRCULAR;
    hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_DeInit(hdma_rx);
    HAL_DMA_Init(hdma_rx);
}
#pragma endregion
