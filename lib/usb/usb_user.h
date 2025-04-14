#ifndef usb_user_h
#define usb_user_h

#include <stm32f4xx_hal.h>
#include "configuration/usbd_conf.h"
#include "middleware/usbd_core.h"
#include "middleware/usbd_ctlreq.h"
#include "middleware/usbd_def.h"
#include "middleware/usbd_ioreq.h"
#include "middleware/usbd_cdc.h"
#include "application/usbd_desc.h"
#include "application/usb_device.h"
#include "application/usbd_cdc_if.h"

typedef void (*FuncRxCallBack)(const uint8_t *const buf, uint32_t length);

typedef struct
{
    uint32_t usb_irq_priority;
    FuncRxCallBack usbRxCB;
} USB_UserParasTypedef;

extern USB_UserParasTypedef USB_UserParas;

void USB_PP_init(uint32_t usb_priority, FuncRxCallBack rxCB);

/** usb 阻塞形式发送数据
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
__inline uint8_t USB_txBlock(uint8_t* Buf, uint16_t Len)
{
    return CDC_Transmit_FS(Buf, Len);
}

#endif