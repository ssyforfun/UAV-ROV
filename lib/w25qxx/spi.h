#ifndef spi_h
#define spi_h

#include <stm32f4xx_hal.h>

void spi_init();
void SPI_SetSpeed(uint8_t prescaler);
uint8_t SPI_ReadWriteByte(uint8_t TxData);

#endif
