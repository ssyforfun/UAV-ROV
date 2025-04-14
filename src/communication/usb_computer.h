#ifndef interface_usb_h
#define interface_usb_h

#include <stm32f4xx_hal.h>
#include "usb_user.h"
#include "comm_recv.h"
#include "cmsis_os.h"
#include "protocol.h"

void usb_init(void);

class USBComm : public CommunicationProtocol, public CommRecv
{
public:
    USBComm();

public:
    virtual uint8_t send(uint8_t *buf, uint32_t length);
	virtual uint8_t send(char *str);
	virtual uint8_t send(uint8_t *cmd, uint8_t *data, uint32_t datalength);
    virtual uint8_t send(char cmd, uint8_t *data, uint32_t datalength);

// ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();
};

extern osThreadId usb_irq_thread_id;
extern osThreadId sendUsb_thread_id;
extern USBComm usbComm;

#endif