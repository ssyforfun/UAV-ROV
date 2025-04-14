/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-09-23 13:05:47
 * @FilePath: \demo3\src\control\gcs_control.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "gcs_control.h"

GCS_input_t gcs_input;

//GCS_ctrl_t gcs_ctrl;
//这个函数需要金生写，就是接收地面站发来的指令，并保存到gcs_ctrl中
//gcs_ctrl的格式和rc_ctrl的格式一样
void gcs_control_init()
{
    gcs_input.gcs_input_mode=Manual;
    gcs_input.gcs_input_status=Disarm;
   
}


void gcs_cmd_collect()  
{
    
}