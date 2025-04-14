/**
    this is for Piezoelectrical control

    include 4 pwm output  --> tim1
                    2 gpio output --> for enable control

    by jinsheng 2019-12-27
**/

#include "gpio_msp.h"

GPIO_InitTypeDef GPIO_InitStruct; // 定义初始化结构体

void GPIO_PINs_Init(void) // 初始化函数
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 使能GPIOA时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // adc
    GPIO_analog_init(GPIOC, 0x3F);                    // PC0-PC5
    GPIO_analog_init(GPIOA, GPIO_PIN_0 | GPIO_PIN_1); // PA0, PA1

    // output - IO/LED PA15, PB0, PB1, PB3, PC13-15
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    GPIO_output_IO_init(GPIOA, GPIO_PIN_15);
    HAL_GPIO_WritePin(GPIOB, 0X000B, GPIO_PIN_RESET);
    GPIO_output_IO_init(GPIOB, 0X000B);
    HAL_GPIO_WritePin(GPIOC, 0XE000, GPIO_PIN_RESET);
    GPIO_output_IO_init(GPIOC, 0XE000);

    // uart
    GPIO_uart_init(GPIOB, GPIO_PIN_6, GPIO_AF7_USART1);
    GPIO_uart_init(GPIOB, GPIO_PIN_7, GPIO_AF7_USART1);
    //GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    // GPIO_InitStruct.Pull = GPIO_PULLUP;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    // GPIO_InitStruct.Pin = GPIO_PIN_6;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    // GPIO_InitStruct.Pin = GPIO_PIN_7;
    // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_uart_init(GPIOA, GPIO_PIN_2, GPIO_AF7_USART2);
    GPIO_uart_init(GPIOA, GPIO_PIN_3, GPIO_AF7_USART2);
    if (USE_I2C == 0)
    {
        GPIO_uart_init(GPIOB, GPIO_PIN_10, GPIO_AF7_USART3);
        GPIO_uart_init(GPIOB, GPIO_PIN_11, GPIO_AF7_USART3);
    }
    GPIO_uart_init(GPIOC, GPIO_PIN_10, GPIO_AF8_UART4);
    GPIO_uart_init(GPIOC, GPIO_PIN_11, GPIO_AF8_UART4);
    GPIO_uart_init(GPIOC, GPIO_PIN_12, GPIO_AF8_UART5);
    GPIO_uart_init(GPIOD, GPIO_PIN_3, GPIO_AF8_UART5);
    GPIO_uart_init(GPIOC, GPIO_PIN_6, GPIO_AF8_USART6);
    GPIO_uart_init(GPIOC, GPIO_PIN_7, GPIO_AF8_USART6);

    // can
    GPIO_can_init(GPIOB, GPIO_PIN_8, GPIO_AF9_CAN1);
    GPIO_can_init(GPIOB, GPIO_PIN_9, GPIO_AF9_CAN1);
    GPIO_can_init(GPIOB, GPIO_PIN_12, GPIO_AF9_CAN2);
    GPIO_can_init(GPIOB, GPIO_PIN_13, GPIO_AF9_CAN2);

    // I2C
    if (USE_I2C == 1)
    {
        GPIO_I2C_init(GPIOB, GPIO_PIN_10, GPIO_AF4_I2C2);
        GPIO_I2C_init(GPIOB, GPIO_PIN_11, GPIO_AF4_I2C2);
    }

    // SPI1 pins
    GPIO_SPI_init(GPIOA, GPIO_PIN_5, GPIO_AF5_SPI1);
    GPIO_SPI_init(GPIOA, GPIO_PIN_6, GPIO_AF5_SPI1);
    GPIO_SPI_init(GPIOA, GPIO_PIN_7, GPIO_AF5_SPI1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_output_IO_init(GPIOA, GPIO_PIN_4);

    // USB nc

    // TIM
    GPIO_pwm_init(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_AF1_TIM1); // TIM1-CH1-CH3
    GPIO_pwm_init(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_AF2_TIM3);               // TIM3-CH1-CH2
    GPIO_pwm_init(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF3_TIM8);               // TIM8-CH3-CH4
    GPIO_pwm_init(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_AF9_TIM12);            // TIM12-CH1-CH2

    // pwm in capture
    GPIO_pwm_init(GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5); // PPM TIM5_CH2 PA1
}

void GPIO_analog_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG; // 配置模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;      // 浮空输入

    GPIO_InitStruct.Pin = GPIO_Pin;         // 配置GPIO_Pin  IO口
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct); // 初始化GPIOx的参数为以上结构体
}

void GPIO_output_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_input_IO_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin)
{
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_pwm_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // note
    // GPIO_AF1_TIM1 = 0x01
    // GPIO_AF1_TIM2 = 0x01
    GPIO_InitStruct.Alternate = AFx;

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_uart_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

        // note
        // GPIO_AF7_USART1 = 0x07
        // GPIO_AF8_LPUART1 = 0x08
        GPIO_InitStruct.Alternate = AFx;

        GPIO_InitStruct.Pin = GPIO_Pin;
        HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_IR_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint32_t mode)
{
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_MCO1_init(void)
{
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void GPIO_SPI_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // required for disconnect detection on SPI encoders
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = AFx;
    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_can_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Alternate = AFx;

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void GPIO_I2C_init(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin, uint8_t AFx)
{
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Alternate = AFx; // AF=GPIO_AF4_I2C2

    GPIO_InitStruct.Pin = GPIO_Pin;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
