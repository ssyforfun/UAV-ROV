/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-09-23 16:14:57
 * @FilePath: \demo3\src\control\control_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include "../fms/fms_interface.h"
#include "pid.h"
#include "main.h"
#include "../ins/ins_interface.h"

#define MOTORS_MAX_NUM_MOTORS 4
#define KV 500.0f           // 500 KV 电机
#define VOTAGE 25.2f
#define THRUST_MAX (KV*VOTAGE)
#define THRUST_MIN (-KV*VOTAGE)

typedef struct
{
  PID_CLASS roll_pid;
  PID_CLASS pitch_pid;
  PID_CLASS yaw_pid;
  PID_CLASS roll_rate_pid;
  PID_CLASS pitch_rate_pid;
  PID_CLASS yaw_rate_pid;
  float roll_thrust;  // roll thrust input value, +/- 1.0
  float pitch_thrust; // pitch thrust input value, +/- 1.0
  float yaw_thrust;   // yaw thrust input value, +/- 1.0
} attitude_controller_t;

typedef struct
{
  PID_CLASS depth_pid;
  PID_CLASS depth_vel_pid;
  float throttle_thrust; // throttle thrust input value, +/- 1.0
  float forward_thrust;  // forward thrust input value, +/- 1.0
  float lateral_thrust;  // lateral thrust input value, +/- 1.0
} z_controller_t;

typedef struct
{
  PID_CLASS xy_pid;
  PID_CLASS xy_vel_pid;
} xy_controller_t;

typedef struct
{
  INS_Bus *ctrl_ins_bus;
  FMS_Bus *ctrl_fms_bus;
  attitude_controller_t att_controller;
  depth_controller_t depth_controller;
} CTRL_Bus;

static void Manual_control();

static void roll_pitch_control(uint32_t set_roll, uint32_t set_pitch);

static void yaw_rate_control(uint32_t set_yaw_rate);

static void yaw_heading_control(uint32_t set_yaw);

static void depth_control(uint32_t set_depth);

static void depth_rate_control(uint32_t set_depth_vel);

static void forward_lateral_control(uint32_t set_forward, uint32_t set_lateral);

static void thrust_alloc();

static void thrust_to_pwm();

// static void thrust_max_init();

static void thrust_zero();

static void control_step();

static void position_control(float forward, float backward, float left, float right);

void control_interface_init(void);

void control_interface_step(uint32_t timestamp);

#endif