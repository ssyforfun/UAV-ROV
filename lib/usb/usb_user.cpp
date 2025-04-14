/************************************************************
 * FileName:        usb_user.cpp
 * Description:     USB User file
 *                  STM32 
 * Auther:          Jinsheng
 * CreateDate:      2022-02-18
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include <usb_user.h>

USB_UserParasTypedef USB_UserParas = {0};

void USB_PP_init(uint32_t usb_priority, FuncRxCallBack rxCB)
{
    /* step 0 : 时钟配置 system_clock_rcc.c
    48MHz = clock_source / M * N /Q
          = CS / M * N / 7 = 336 / 7 = 48
    */

    /* step 1 : userISR.c设置中断
    extern PCD_HandleTypeDef hpcd_USB_OTG_FS; // 在usbd_conf.c里定义的变量
    void OTG_FS_IRQHandler(void)
    { 
        HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);  //后期需要改成在 任务里处理 HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS)，采用 semphomore的方式
    }    
    */

    //* step2 : 初始化
    USB_UserParas.usbRxCB = rxCB;
    USB_UserParas.usb_irq_priority = usb_priority; // ISR_Priority::USB_Prior; // 配置好usb的中断优先级
    MX_USB_DEVICE_Init();  // 在usb_device.c里

    /* step3 : 间接调用收发
    rxCB -> CDC_Receive_FS
    txBlock -> CDC_Transmit_FS
    */
}



