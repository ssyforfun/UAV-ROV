#include "fms_interface.h"
// #include "gcs_control.h"
#include "math.h"
extern RC_input_t rc_input;
extern GCS_input_t gcs_input;
extern INS_Bus ins_bus;
extern bool rc_updated;
float yaw_feedback;
float yaw_rate_desired;
FMS_Bus fms_bus;
float depth_feedback;
float alt_feedback;
shell_priority_list shell_priority = Cooperate;
mode_list last_mode;
mode_list cur_mode;
float depth_desired = 0.0f;
float alt_desired = 0.0f;
float yaw_desired = 0.0f;
float last_depth_cmd = 0.0f;
float last_yaw_cmd = 0.0f;
float gps_feedback[3] = {0.0, 0.0, 0.0};
float rollstick = 0.0f;
float pitchstick = 0.0f;
float throttlestick = 0.0f;
float yawstick = 0.0f;
// float forwardstick = 0.0f;
// float lateralstick = 0.0f;
float forwardout = 0.0f;
float lateralout = 0.0f;
float zout = 0.0f;
float rollout = 0.0f;
float pitchout = 0.0f;
float yawout = 0.0f;

uint32_t dt_ctrl=0;
float roll_feedback=0.0f;
float pitch_feedback=0.0f;
float roll_rate_feedback=0.0f;
float pitch_rate_feedback=0.0f;
float yaw_rate_feedback=0.0f;
float roll_desired=0.0f;
float pitch_desired=0.0f;
int step_delay_count = 0;
const int STEP_TIME_DELAY = 50;  //阶跃延迟时间 = STEP_TIME_DELAY * 0.1s
//定深控制时需要设置一个定深油门的基准值，在这个值下，四旋翼浮力与重力基本平衡，从而基本能够悬浮，深度环的输出叠加在这个油门基础值上，就可以使其上下机动。
TimeTag fms_interval = {
    .tag = 0,
    .period = 100,
};

#define constrain_float(val, min_val, max_val) (val < min_val ? min_val : (val > max_val ? max_val : val))

static float norm_input(int16_t radio_in)
{
  float ret;
  if (radio_in < JOYSTICK_READ_NEUTRAL)
  {
    if (JOYSTICK_READ_MIN >= JOYSTICK_READ_NEUTRAL)
    {
      return 0.0f;
    }
    ret = (radio_in - JOYSTICK_READ_NEUTRAL) / (JOYSTICK_READ_NEUTRAL - JOYSTICK_READ_MIN);
  }
  else
  {
    if (JOYSTICK_READ_MAX <= JOYSTICK_READ_NEUTRAL)
    {
      return 0.0f;
    }
    ret = (radio_in - JOYSTICK_READ_NEUTRAL) / (JOYSTICK_READ_MAX - JOYSTICK_READ_NEUTRAL);
  }
  return constrain_float(ret, -1.0f, 1.0f);
}

// static float Limit(float pwm, float min, float max)
// {
//   return pwm < min ? min : (pwm > max ? max : pwm);
// }

static float absself(float num)
{
  if (num > 0)
  {
    return num;
  }
  else
  {
    return -num;
  }
}
// static float deadzone_range(float num1, float num2)//1000-2000
// {
//     if( absself(num1 - JOYSTICK_READ_NEUTRAL ) < num2)
//     {
//         return 0;
//     }
//     else{
//         return num1;
//     }
// }

static float deadzone_range(float num1, float num2) //-1 - 1
{
  if (absself(num1 - JOYSTICK_READ_NEUTRAL1) < num2)
  {
    return 0;
  }
  else
  {
    return num1;
  }
}

// 在这一步将遥控器或者GCS转换，遥控器in , 期望out
static void FMS_step(void);

void fms_interface_init()
{
  // 初始化遥控器和上位机控制
  rc_control_init();

  gcs_control_init();
}

static uint32_t last_timestamp;

// 数据传输函数
void fms_interface_step(uint32_t timestamp)
{
  // 遥控器的优先级高
  if (rc_updated)
  {
    fms_bus.fms_rc_input = &rc_input;
    fms_bus.fms_rc_input->timestamp = timestamp;
  }
  else
  {
    fms_bus.fms_gcs_input = &gcs_input;
    fms_bus.fms_gcs_input->timestamp = timestamp;
  }

  fms_bus.fms_ins_bus = &ins_bus;

  FMS_step();
}



// 模式判断函数
static void FMS_step(void)
{
  if (rc_updated)
  {
    if (fms_bus.fms_rc_input->sw1 == RC_SW_UP)
    {
      fms_bus.fms_out.status = Arm;
    }
    else
    {
      fms_bus.fms_out.status = Disarm;
    }

    if (fms_bus.fms_rc_input->sw2 == RC_SW_UP && fms_bus.fms_rc_input->sw4 == RC_SW_UP)
    {
      fms_bus.fms_out.mode = Stabilize;
    }
    else if (fms_bus.fms_rc_input->sw2 == RC_SW_UP && fms_bus.fms_rc_input->sw4 == RC_SW_DOWN)
    {
      fms_bus.fms_out.mode = Auto;
    }
    else if (fms_bus.fms_rc_input->sw2 == RC_SW_DOWN && fms_bus.fms_rc_input->sw4 == RC_SW_UP)
    {
      fms_bus.fms_out.mode = Zhold;
    }
    else if (fms_bus.fms_rc_input->sw2 == RC_SW_DOWN && fms_bus.fms_rc_input->sw4 == RC_SW_DOWN)
    {
      fms_bus.fms_out.mode = Manual;
    }

    // 摄像头舵机控制
    if (fms_bus.fms_rc_input->sw3 == RC_SW_UP)
    {
      fms_bus.fms_out.servo_cmd = 1;
    }
    else if (fms_bus.fms_rc_input->sw3 == RC_SW_MID)
    {
      fms_bus.fms_out.servo_cmd = 0;
    }
    else if (fms_bus.fms_rc_input->sw3 == RC_SW_DOWN)
    {
      fms_bus.fms_out.servo_cmd = -1;
    }

    // 处理摇杆输入
    rollstick = deadzone_range(fms_bus.fms_rc_input->ch1, 0.05);
    pitchstick = deadzone_range(fms_bus.fms_rc_input->ch2, 0.05);
    throttlestick = deadzone_range(fms_bus.fms_rc_input->ch3, 0.05);
    yawstick = deadzone_range(fms_bus.fms_rc_input->ch4, 0.05);
    // forwardstick = deadzone_range(fms_bus.fms_rc_input->ch2, 0.05);
    // lateralstick = deadzone_range(fms_bus.fms_rc_input->ch1, 0.05);
  }

  // break;

  // case Cooperate: // 同时接收遥控器和shell的指令，若实现太复杂或没什么用就删掉
  //   // 处理合作模式
  //   break;

  // case FullControl: // 作上位机控制
  //   // rollstick = fms_bus.fms_gcs_input->row_cmd;
  //   // pitchstick = fms_bus.fms_gcs_input->pitch_cmd;
  //   // throttlestick = fms_bus.fms_gcs_input->depth_cmd;
  //   // yawstick = fms_bus.fms_gcs_input->yaw_cmd;
  //   // // forwardstick = fms_bus.fms_gcs_input->forward_cmd;
  //   // // lateralstick = fms_bus.fms_gcs_input->lateral_cmd;

  //   // fms_bus.fms_out.status = fms_bus.fms_gcs_input->gcs_input_status;
  //   // fms_bus.fms_out.mode = fms_bus.fms_gcs_input->gcs_input_mode;
  //   break;

  // default:
  //   // 如果shell_priority没有匹配的值，可以加入默认行为
  //   break;
  // }

  //cur_mode = fms_bus.fms_out.mode;

  if (fms_bus.fms_out.mode == Manual)
  {
    // forwardout = forwardstick * Speed_x_Max;
    // lateralout = lateralstick * Speed_y_Max;
    zout = throttlestick * Speed_z_Max;
    rollout = rollstick * Roll_Rate_Max;
    pitchout = pitchstick * Pitch_Rate_Max;
    yawout = yawstick * Yaw_Rate_Max;

    // fms_bus.fms_out.u_cmd = forwardout;
    // fms_bus.fms_out.v_cmd = lateralout;
    fms_bus.fms_out.w_cmd = zout;
    fms_bus.fms_out.p_cmd = rollout;
    fms_bus.fms_out.q_cmd = pitchout;
    fms_bus.fms_out.r_cmd = yawout;
  }
  else if (fms_bus.fms_out.mode == Auto) //这里要设计航迹
  {
    fms_bus.fms_out.x_cmd = x_setpoint;
    fms_bus.fms_out.y_cmd = y_setpoint;
    fms_bus.fms_out.z_cmd = z_setpoint;
  }
  else
  {
    dt_ctrl = (fms_bus.fms_rc_input->timestamp >= last_timestamp) ? (fms_bus.fms_rc_input->timestamp - last_timestamp) : (0xFFFFFFFF - last_timestamp + fms_bus.fms_rc_input->timestamp);
    last_timestamp = fms_bus.fms_rc_input->timestamp;

    // 传感器角度数据
    roll_feedback = fms_bus.fms_ins_bus->imu->ang_roll;
    pitch_feedback = fms_bus.fms_ins_bus->imu->ang_pitch;
    yaw_feedback = fms_bus.fms_ins_bus->imu->ang_yaw;

    // 传感器角速度数据
    roll_rate_feedback = fms_bus.fms_ins_bus->imu->gyr_x;
    pitch_rate_feedback = fms_bus.fms_ins_bus->imu->gyr_y;
    yaw_rate_feedback = fms_bus.fms_ins_bus->imu->gyr_z;

    if(yaw_feedback>180)
    {
      yaw_feedback = yaw_feedback - 360;
      yaw_rate_feedback = - yaw_rate_feedback;
    }

    // 深度反馈
    depth_feedback = fms_bus.fms_ins_bus->bar->depth;

    //高度反馈
    alt_feedback = fms_bus.fms_ins_bus->alt->altitude;

    //GPS反馈
    gps_feedback[0] = fms_bus.fms_ins_bus->gps->global_position[0];
    gps_feedback[1] = fms_bus.fms_ins_bus->gps->global_position[1];
    gps_feedback[2] = fms_bus.fms_ins_bus->gps->global_position[2];
    // row pitch x y yaw_rate
    // todo：事实上roll，pitch在目前构型下可不考虑闭环控制，因为没有xy位置控制，一旦有roll和pitch，ROV会快速撞向水池边缘
    roll_desired = rollstick * Roll_Max;
    pitch_desired = pitchstick * Pitch_Max;
    // float yaw_desired = yawstick * Yaw_Max;
    yaw_rate_desired = yawstick * Yaw_Rate_Max;
    alt_desired += throttlestick * Speed_z_Max * 0.001 * dt_ctrl;
    // forwardout = forwardstick * Speed_x_Max; // x方向采用手动
    // lateralout = lateralstick * Speed_y_Max; // y方向采用手动

    fms_bus.fms_out.phi_cmd = roll_desired;
    fms_bus.fms_out.theta_cmd = pitch_desired;
    fms_bus.fms_out.r_cmd = yaw_rate_desired;
    fms_bus.fms_out.z_cmd = alt_desired;
  }
}