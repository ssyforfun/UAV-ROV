/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-10-28 13:17:23
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-11-29 14:39:02
 * @FilePath: \demo3\src\control\init_actuator.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "init_actuator.h"
#include "../../include/tim.h"
#include "main.h"



void init_actuator()
{
    pwm_speed_set(0, 0);
    pwm_speed_set(1, 0);
    pwm_speed_set(2, 0);
    pwm_speed_set(3, 0);
    pwm_speed_set(4, 0);
    pwm_speed_set(5, 0);
    pwm_speed_set(6, 0);
    pwm_speed_set(7, 0);
    pwm_speed_set(8, 0);
    osDelay(5000);

    pwm_speed_set(0, 0.5);
    pwm_speed_set(1, 0.5);
    pwm_speed_set(2, 0.5);
    pwm_speed_set(3, 0.5);
    pwm_speed_set(4, 0.5);
    pwm_speed_set(5, 0.5);
    pwm_speed_set(6, 0.5);
    pwm_speed_set(7, 0.5);
    pwm_speed_set(8, 0.5);
    osDelay(1000);
    
    pwm_speed_set(0, 0);
    pwm_speed_set(1, 0);
    pwm_speed_set(2, 0);
    pwm_speed_set(3, 0);
    pwm_speed_set(4, 0);
    pwm_speed_set(5, 0);
    pwm_speed_set(6, 0);
    pwm_speed_set(7, 0);
    pwm_speed_set(8, 0);
}