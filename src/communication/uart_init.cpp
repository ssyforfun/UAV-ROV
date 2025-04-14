/** ******************** 使用方法 *************************
 * 1 根据串口号，添加文件 uartx_msp.cpp 文件，并配置该文件
 * 2 在 uart_init.cpp 里声明 串口对应的 句柄和初始化函数， 如:
 *      -- extern UART_HandleTypeDef *huart3Handle;
 *      -- extern void (*uart3_init)(void);
 * 3 在 uart_init 里初始化硬件外设 和 接口类
 * *******************************************************/

#include "uart_init.h"
#include "main.h"
#include "FreeRTOS_Shell.h"

extern UART_HandleTypeDef *huart1Handle;
extern void (*uart1_init)(void);
extern UART_HandleTypeDef *huart2Handle;
extern void (*uart2_init)(void);
extern UART_HandleTypeDef *huart3Handle;
extern void (*uart3_init)(void);
extern UART_HandleTypeDef *huart4Handle;
extern void (*uart4_init)(void);
extern UART_HandleTypeDef *huart5Handle;
extern void (*uart5_init)(void);
extern UART_HandleTypeDef *huart6Handle;
extern void (*uart6_init)(void);

// extern UART_HandleTypeDef *huart6Handle;
// extern void (*uart6_init)(void);

CommunicationProtocol *userComm;
UartComputer uartComputer; // uart6
UartIMU uartImu;           // uart2
UartAtkTof uartTof;        // uart1
// UartWifi uartWifi;      // uart4
// UartSbus uartSbus;      // uart5
// UartComputer uart485; // uart6

#ifdef USE_UESR_SHELL
void FreeRtosShellSendInst(uint8_t *buf, int length)
{
    // if ((length > 255) || (length < 0))
    //     return;
    // char sbuf[256];
    // for (int i = 0; i < length; i++)
    // {
    //     sbuf[i] = buf[i];
    // }
    // sbuf[length] = 0;
    // userComm->send(sbuf);

    userComm->send('x', buf, length); // cmd没有任何意义 _isFrameData = false;的时候
}
#endif

void uart_init(void)
{
#ifdef USE_UESR_SHELL
    osThreadDef(shellTask, FreeRTOS_Shell, osPriority::osPriorityNormal, 0, 1024);
    osThreadCreate(osThread(shellTask), NULL);
    // extern void (*FreeRtosShellSend)(uint8_t *buf, int length);
    FreeRtosShellSend = FreeRtosShellSendInst;
#ifndef USE_USER_COM
    uartComputer._isFrameData = false;
    usbComm._isFrameData = false;
#endif
#endif

    // 初始化硬件外设
    (*uart1_init)();
    (*uart2_init)();
    // (*uart3_init)();
    //(*uart4_init)();
    //(*uart5_init)();
    (*uart6_init)();

    // 初始化接口类
    userComm = &uartComputer;
    uartComputer.init(1024, huart6Handle);
    uartComputer.setDirConfig(GPIOB, GPIO_PIN_1, true);

    uint8_t imuTxHeader[2] = {0x55, 0xAF};
    uartImu.init(imuTxHeader, 256, huart2Handle);

    uartTof.init(256, huart1Handle);

    // uart485.init(1024, huart6Handle);
}
