#ifndef FMS_INTERFACE_H
#define FMS_INTERFACE_H

#include "rc_control.h"
#include "gcs_control.h"
#include "../../include/main.h"
#include "../ins/ins_interface.h"


#define JOYSTICK_READ_MIN       1000
#define JOYSTICK_READ_MAX       2000
#define JOYSTICK_READ_NEUTRAL   1500
#define JOYSTICK_OUTPUT_MIN     -1.0f
#define JOYSTICK_OUTPUT_MAX     1.0f

#define JOYSTICK_READ_MIN1       -1.0f
#define JOYSTICK_READ_MAX1       1.0f
#define JOYSTICK_READ_NEUTRAL1   0.0f


/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((int)1)
#define RC_SW_MID               ((int)0)
#define RC_SW_DOWN              ((int)-1)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

//手动最大值
#define Speed_x_Max 1.5
#define Speed_y_Max 1.5
#define Speed_z_Max 1.0

//自动pid最大值
#define Speed_x_Max_pid 0.5
#define Speed_y_Max_pid 0.5
#define Speed_z_Max_Pid 0.5

#define Acc_z_Max 0.1

//手动最大值
#define Roll_Max 0.523
#define Pitch_Max 0.523
#define Yaw_Max 3.142
#define Yaw_Rate_Max 0.785
#define Roll_Rate_Max 0.523
#define Pitch_Rate_Max 0.523

//自动pid最大值
#define Roll_Max_pid 20
#define Pitch_Max_pid 20
#define Yaw_Max_pid 90
#define Yaw_Rate_Max_pid 30
#define Roll_Rate_Max_pid 10
#define Pitch_Rate_Max_pid 10

typedef struct {
  /* fms output timestamp */
  uint32_t timestamp;

  /* roll rate command in body frame */
  float p_cmd;

  /* pitch rate command in body frame */
  float q_cmd;

  /* yaw rate command in body frame */
  float r_cmd;

  /* roll command in body frame */
  float phi_cmd;

  /* pitch command in body frame */
  float theta_cmd;

  /* yaw command in body frame */
  float psi_cmd;

  /* velocity x command in control frame */
  float u_cmd;

  /* velocity y command in control frame */
  float v_cmd;

  /* velocity z command in control frame */
  float w_cmd;

  /* position z command in control frame */
  float z_cmd;

  /* position x command in control frame */
  float x_cmd;

  /* position y command in control frame */
  float y_cmd;
  /* 摄像头舵机 */
  float servo_cmd;

  status_list status;

  mode_list mode;
  /* home position [x y h yaw], unit [m m m rad] */
  // uint32_t home[4];

  
} FMS_Out;

typedef struct
{
  RC_input_t *fms_rc_input;
  GCS_input_t *fms_gcs_input;
  const INS_Bus *fms_ins_bus;
  FMS_Out fms_out;
} FMS_Bus;

void fms_interface_init();
void fms_interface_step(uint32_t timestamp);

#endif