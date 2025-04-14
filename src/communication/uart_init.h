/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-09-23 11:18:52
 * @FilePath: \demo3\src\communication\uart_init.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _UART_INIT_H
#define _UART_INIT_H

#include "uart_computer.h"
#include "uart_miniSVP.h"
#include "uart_imu.h"
#include "uart_atkTOF.h"

void uart_init(void);
extern CommunicationProtocol *userComm;
extern UartComputer uartComputer;
extern UartIMU uartImu; 
extern UartComputer uart485; // uart6
extern UartAtkTof uartTof;  // uart1

#endif