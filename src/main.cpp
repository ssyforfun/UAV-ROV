/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-09-18 14:07:03
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-18 14:14:53
 * @FilePath: \demo2\src\main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <main.h>

/** 外设使用情况 
 * HSE_VALUE = 8000000; // HSE_VALUE is set in stm32f4xx_hal_conf.h, default is 25MHz, using -D HSE_VALUE=8000000UL at platform.ini predefined
 * System_Clock 168MHz, APB1=42MHz, APB2=84MHz
 * ADC      ADC1, 8pins: PC0-PC3, PA0, PA1, PC4, PC5
 * USART    USART1([PB6]=TX1, [PB7]=RX1) // computer                    TOF
 *          USART2([PA2]=TX2, [PA3]=RX2) // imu                         imu
 *          USART3([PB10]=TX3, [PB11]=RX3) // modbus    
 *          UART5([PC12]=TX5, [PD2]=RX5) // sbus
 *          USART6([PC6]=TX6, [PC7]=RX6) // 485                         computer
 * CAN      CAN1([PB9]=TX1, [PB8]=RX1)
 *          CAN2([PB13]=TX2, [PB12]=RX2)
 * I2C      I2C2([PB10]=SCL2, [PB11]=SDA2) 与 USART3 共用引脚            水压计
 * SPI      SP1([PA4]=NSS, [PA5]=SCK, [PA6]=MISO, [PA7]=MOSI) 
 * USB      ([PA11]=DM, [PA12]=DP)
 * TIM      TIM1([PA8]=CH1, [PA9]=CH2, [PA10]=CH3)                      motor 4 3 2 
 *          TIM3([PB4]=CH1, [PB5]=CH2)                                  motor 0 1
 *          TIM8([PC8]=CH3, [PC9]=CH4)                                  motor 5 6
 *          TIM12([PB14]=CH1, [PB15]=CH2)                               servo motor 7
 *          TIM14(for HAL_TICK)
 * LED      ([PC15]=LED1, [PC14]=LED2, [PC13]=LED3, [PB3]=LED4, [PB0]=LED5, [PA15]=LED6)
 * IO       ([PB1]=IO0)
 * DMA      overall DMA1 stream0-7 all used by uart, DMA2 stream0, 1, 5, 6, 7 is used. only remained dma2 stream 2, 3, 4
 *          DMA2_Stream5(USART1_RX with DMA_Channel4)
 *          DMA2_Stream7(USART1_TX with DMA_Channel4)
 *          DMA1_Stream5(USART2_RX with DMA_Channel4)
 *          DMA1_Stream6(USART2_TX with DMA_Channel4)
 *          DMA1_Stream1(USART3_RX with DMA_Channel4)
 *          DMA1_Stream3(USART3_TX with DMA_Channel4)
 *          DMA1_Stream2(USART4_RX with DMA_Channel4)
 *          DMA1_Stream4(USART4_TX with DMA_Channel4)
 *          DMA1_Stream0(USART5_RX with DMA_Channel4)
 *          DMA1_Stream7(USART5_TX with DMA_Channel4)
 *          DMA2_Stream1(USART6_RX with DMA_Channel5)
 *          DMA2_Stream6(USART6_TX with DMA_Channel5)
 *          DMA2_Stream0(ADC1 with DMA_channel0)
**/

osThreadId led_thread_id;
osThreadId uart_thread_id;
osThreadId setup_thread_id;
osThreadId dataAquisition_thread_id;
osThreadId main_thread_id;
int main(int argc, char **argv)
{
  //iwatchdog_init();
  
  HAL_Init();

  // HSE=8MHz, CPU=168MHz
  // 要修改HSE_VALUE的值（在stm32f4xx_hal_conf.h）文件里
  //SystemClockHSI_PLL_Config();
  SystemClockHSE_PLL_Config();

  GPIO_PINs_Init();

  osThreadDef(ledControl, vTaskLedBreathing, osPriority::osPriorityIdle, 0, 256);
  led_thread_id = osThreadCreate(osThread(ledControl), NULL);

  osThreadDef(setup, vThreadSetup, osPriority::osPriorityLow, 0, 1024);
  setup_thread_id = osThreadCreate(osThread(setup), NULL);

  osKernelStart();


}


