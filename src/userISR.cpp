
#include "userISR.h"
#include <main.h>

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/
extern "C"
{
#ifdef userSysTick_h
	void TIMBASE_IRQHandler(void)
	{
		if ((TIM_TIMEBASE->SR & TIM_SR_UIF_Msk) == TIM_SR_UIF_Msk)
		{ // 代表 TIM14 发生中断，这里用作 非freeRTOS 的 systick
			// 替代 SysTick_Handler 中断
			// TIM_TIMEBASE->SR &= (~TIM_SR_UIF_Msk); // 0x00000001
			TIM_TIMEBASE->SR &= 0xFFFE; // 用此语句替换TIM_TIMEBASE->SR &= (~TIM_SR_UIF_Msk), 可以节省时间
			// HAL_IncTick();
			uwTick += uwTickFreq; // 用此语句替代HAL_IncTick();可以节省一些进出函数的时间
#ifdef watchdog_h
			if (watchdogEnabled)
				IWDG_Refresh();
#endif
		}
		else
		{
			TIM_TIMEBASE->SR = 0;
		}
	}
#endif

	void TIM1_UP_TIM10_IRQHandler(void)
	{
		TIM1->SR &= ~TIM_SR_UIF;
	}

    void TIM5_IRQHandler(void)
	{
		input_caputure_pwm_callback();
	}

	void EXTI15_10_IRQHandler(void)
	{
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	}

	void EXTI9_5_IRQHandler(void)
	{
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
		if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
	}

	void EXTI0_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	}
	void EXTI1_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	}
	void EXTI2_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
	}
	void EXTI3_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
	}
	void EXTI4_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
	}

#ifdef _ADC_H
	// void DMA2_Stream0_IRQHandler(void)
	void DMA_ADC_IRQHandler(void)
	{
		ADC_DMA_callback();
	}
#endif

#ifdef interface_usb_h
	// extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
	extern osSemaphoreId sem_usb_irq;
	void OTG_FS_IRQHandler(void)
	{
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
		osSemaphoreRelease(sem_usb_irq);
		return;
		// HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
	}
#endif

#ifdef can_h
	void CAN1_RX0_IRQHandler(void)
	{
		// HAL_CAN_IRQHandler(&CanHandle);
		can1RxCallback();
	}
	void CAN1_RX1_IRQHandler(void)
	{
		// HAL_CAN_IRQHandler(&CanHandle);
		can1RxCallback();
	}
	void CAN2_RX0_IRQHandler(void)
	{
		// HAL_CAN_IRQHandler(&CanHandle);
		can2RxCallback();
	}
	void CAN2_RX1_IRQHandler(void)
	{
		// HAL_CAN_IRQHandler(&CanHandle);
		can2RxCallback();
	}
#endif
}
