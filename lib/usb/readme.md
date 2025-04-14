# USB CDC
  usb cdc配置成虚拟串口(VCP, virtual comport)
  基本上通过stm32cubeMX自动生成，然后调用usbd_cdc_if.c的两个函数收发，CDC_Receive_FS从中断进入，CDC_Transmit_FS发送block模式

# USB device for CDC(VPC, vitual comport)
 ## USB 结构： https://blog.csdn.net/king_jie0210/article/details/76713938
  ### 底层：hal driver
    - stm32f4xx_hal_pcd.c
    - stm32f4xx_hal_pcd_ex.c
    - stm32f4xx_ll_usb.c
  ### 配置层：configuration
    - usbd_conf.c
  ### 中间层；middleware
    - usb_core.c
    - usb_ctlreq.c
    - usb_ioreq.c
    - usb_cdc.c
  ### 应用层：application
    - usb_device.c
    - usb_cdc_if.c
    - usb_desc.c
    - others
 ## USB 步骤：
 ## USB 文件修改[stm32CubeMX生成的代码为参考]
  ### 配置层
    1 usbd_def.h
      a #include "usbd_conf.h"  修改成 #include "configuration/usbd_conf.h" 
    2 usbd_conf.h
      - 去除 #include "main.h"
    3 usbd_conf.c
      a 修改 include的某些文件的路径
      b 增加 #include "usb_user.h"
      c HAL_PCD_MspInit函数里，把HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0); 改成 HAL_NVIC_SetPriority(OTG_FS_IRQn, USB_UserParas.usb_irq_priority, 0);
  ### 中间层
    1 usb_core.h 
      a 修改 include的某些文件的路径
    2 usb_ctrlreq.h usb_ioreq.h usb_cdc.h
      - no change
    3 usb_core.c usb_ctrlreq.c usb_ioreq.c usb_cdc.c
      - no change
   #### 如果要像odrive一样，免驱的话，需要修改usb_cdc.c和增加一个接口(包含两个端口in/out)
  ### 应用层
    1 usb_device.h usbd_desc.h usbd_cdc_if.h
      a 修改 include的某些文件的路径
    2 usb_device.c usbd_desc.c 
      a 修改 include的某些文件的路径
    3 usb_cdc_if.c
      a 增加 #include "usb_user.h"
      b 在 CDC_Receive_FS 函数里增加：
        if (USB_UserParas.usbRxCB != NULL)
          USB_UserParas.usbRxCB(Buf, *Len);

# USB issue
 ## 无法发送整数倍字节
  https://blog.csdn.net/flydream0/article/details/53205286


