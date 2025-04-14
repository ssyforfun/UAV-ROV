/**

**/

#ifndef _GPIO_MSP_H
#define _GPIO_MSP_H

#include <stm32f4xx_hal.h>
#include <pinConfig.h>

#define USE_I2C (0) // 1: use I2C, 0: not use I2C

void GPIO_PINs_Init(void);
void GPIO_analog_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_output_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_input_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
void GPIO_pwm_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_uart_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_IR_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint32_t mode);
void GPIO_MCO1_init(void);
void GPIO_SPI_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_can_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);
void GPIO_I2C_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx);

#endif
