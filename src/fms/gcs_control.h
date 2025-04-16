/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-12-16 16:42:41
 * @FilePath: \demo3\src\fms\gcs_control.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef GCS_CONTROL_H
#define GCS_CONTROL_H
#include "stm32f4xx_hal.h" 


//bool gcs_updated;

typedef enum
{
    Disarm = 0,
    Arm,
   
} status_list;

typedef enum
{       
  Stabilize = 0,  
  Auto,
  Zhold,
  Manual,            
} mode_list;

typedef enum
{
  FullControl = 0,
  Cooperate,
  ShellDisable,
} shell_priority_list;

typedef struct
{
    //manual 模式开环指令,实现方法待定
    float row_cmd;
    float pitch_cmd;
    float yaw_cmd;
    float depth_cmd;
    // float forward_cmd;
    // float lateral_cmd;
    //自动模式指令，目前xy无法指定
    float yaw_disired_cmd;
    float depth_disired_cmd;

    uint32_t timestamp;
    status_list gcs_input_status;
    mode_list gcs_input_mode;
} GCS_input_t;


void gcs_control_init();

void gcs_cmd_collect();

#endif