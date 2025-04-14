#include "spi.h"

static SPI_HandleTypeDef spiHandle;

void spi_init()
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    spiHandle.State = HAL_SPI_STATE_RESET;
    spiHandle.Instance = SPI1;
    spiHandle.Init.Mode = SPI_MODE_MASTER;
    spiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    spiHandle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spiHandle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spiHandle.Init.NSS = SPI_NSS_SOFT;
    spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // SPI1 for APB2 = sysclock/2
    spiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHandle.Init.CRCPolynomial = 7;

    HAL_SPI_Init(&spiHandle);
    __HAL_SPI_ENABLE(&spiHandle);

    SPI_ReadWriteByte(0xff); // 启动传输
}

// APB2时钟频率是84MHz
// speed = 84MHz / prescaler
// 当有数据传输时不能改变速率
void SPI_SetSpeed(uint8_t prescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(prescaler)); // 判断有效性
    SPI1->CR1 &= 0XFFC7;                                // 位3-5清零，用来设置波特率
    SPI1->CR1 |= prescaler;                             // 设置SPI1速度
    //__HAL_SPI_ENABLE(&spiHandle);                       // 使能SPI1
}

// SPI1 读写一个字节
// TxData:要写入的字节
// 返回值:读取到的字节
uint8_t SPI_ReadWriteByte(uint8_t TxData)
{ // 参考 HAL_SPI_TransmitReceive() 函数

    // 等待发送区空
    while (!__HAL_SPI_GET_FLAG(&spiHandle, SPI_FLAG_TXE))
    {
    }
    *(__IO uint8_t *)&spiHandle.Instance->DR = TxData; // 通过外设SPIx发送一个byte  数据

    // 等待接收完一个byte
    while (!__HAL_SPI_GET_FLAG(&spiHandle, SPI_FLAG_RXNE))
    {
    } 
    return (uint8_t)spiHandle.Instance->DR;
}
