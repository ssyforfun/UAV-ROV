/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-10-28 13:02:51
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-12-09 15:40:14
 * @FilePath: \demo3\src\control\send_actuator_cmd.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "send_actuator_cmd.h"
#include "../../include/tim.h"
#include "control_interface.h"

extern float motor_out[MOTORS_MAX_NUM_MOTORS];
void send_actuator_cmd()
{
    pwm_speed_set(0, motor_out[0]);
    pwm_speed_set(1, -motor_out[1]);
    pwm_speed_set(2, motor_out[2]);
    pwm_speed_set(3, motor_out[3]);
    pwm_speed_set(4, -motor_out[4]);
    pwm_speed_set(5, -motor_out[5]);
    pwm_speed_set(6, motor_out[6]);
    pwm_speed_set(7, -motor_out[7]);

//     pwm_speed_set(0, 0);
//     pwm_speed_set(1, 0);
//     pwm_speed_set(2, 0);
//     pwm_speed_set(3, 0);
//     pwm_speed_set(4, 0.1);
//     pwm_speed_set(5, 0.1);
//     pwm_speed_set(6, 0.1);
//     pwm_speed_set(7, 0.1);
 }