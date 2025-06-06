#include "control_interface.h"
#include <math.h>

#ifndef _constrain
#define _constrain(amt, low, high) ((amt) = ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))))
#endif

#define MOTORS_MOT_1 0U
#define MOTORS_MOT_2 1U
#define MOTORS_MOT_3 2U
#define MOTORS_MOT_4 3U
// #define MOTORS_MOT_5 4U
// #define MOTORS_MOT_6 5U
// #define MOTORS_MOT_7 6U
// #define MOTORS_MOT_8 7U

#define MOTORS_DEADZONE_P 0.0f
#define MOTORS_DEADZONE_N 0.0f
#define CONSTANT_G 9.81f

#define WEIGHT 1.5f
// 推力系数矩阵 roll pitch yaw heave surge sway，6推进器
// float motor1_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, 1.0f, 0, -1.0f, 1.0f};
// float motor2_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, -1.0f, 0, -1.0f, -1.0f};
// float motor3_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, -1.0f, 0, 1.0f, 1.0f};
// float motor4_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, 1.0f, 0, 1.0f, -1.0f};
// float motor5_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, 0, 0, -1.0f, 0, 0};
// float motor6_factor[MOTORS_MAX_NUM_MOTORS] = {-1.0f, 0, 0, -1.0f, 0, 0};

// 推力系数矩阵 roll pitch yaw heave surge sway，8推进器
// float motor1_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, -1.0f, 0, -1.0f, 1.0f};
// float motor2_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, 1.0f, 0, -1.0f, -1.0f};
// float motor3_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, 1.0f, 0, 1.0f, 1.0f};
// float motor4_factor[MOTORS_MAX_NUM_MOTORS] = {0, 0, -1.0f, 0, 1.0f, -1.0f};
// float motor5_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, 1.0f, 0, -1.0f, 0, 0};   // 右前
// float motor6_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, -1.0f, 0, -1.0f, 0, 0};  // 右后
// float motor7_factor[MOTORS_MAX_NUM_MOTORS] = {-1.0f, 1.0f, 0, -1.0f, 0, 0};  // 左前
// float motor8_factor[MOTORS_MAX_NUM_MOTORS] = {-1.0f, -1.0f, 0, -1.0f, 0, 0}; // 左后

// X型四旋翼 thrust roll pitch yaw
float motor1_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, 0.707f, 0.707f, 1.0f};
float motor2_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, 0.707f, -0.707f, -1.0f};
float motor3_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, -0.707f, -0.707f, 1.0f};
float motor4_factor[MOTORS_MAX_NUM_MOTORS] = {1.0f, -0.707f, 0.707f, -1.0f};

extern FMS_Bus fms_bus;
extern INS_Bus ins_bus;

CTRL_Bus ctrl_bus;

float motor_out[MOTORS_MAX_NUM_MOTORS];
float motor_thrust[MOTORS_MAX_NUM_MOTORS];
// float thrust_max;



TimeTag ctrl_interval = {
    .tag = 0,
    .period = 100,
};

float limitThrust(float value)
{
    if (value > THRUST_MAX)
    {
        value = THRUST_MAX;
    }
    else if (value < THRUST_MIN)
    {
        value = THRUST_MIN;
    }
    return value;
}


void control_interface_init(void)
{
    // thrust_max_init();
    // 初始化姿态控制器PID
    ctrl_bus.att_controller.roll_pid.init();
    ctrl_bus.att_controller.roll_pid.isIncrement = true;
    ctrl_bus.att_controller.roll_pid.vcmax = Roll_Rate_Max;
    ctrl_bus.att_controller.roll_pid.vcmin = -Roll_Rate_Max;
    ctrl_bus.att_controller.roll_pid.kp = 0;
    ctrl_bus.att_controller.roll_pid.ki = 0;
    ctrl_bus.att_controller.roll_pid.kd = 0;
    ctrl_bus.att_controller.pitch_pid.init();
    ctrl_bus.att_controller.pitch_pid.isIncrement = true;
    ctrl_bus.att_controller.pitch_pid.vcmax = Pitch_Rate_Max;
    ctrl_bus.att_controller.pitch_pid.vcmin = -Pitch_Rate_Max;
    ctrl_bus.att_controller.pitch_pid.kp = 0.00005;
    ctrl_bus.att_controller.pitch_pid.ki = 0;
    ctrl_bus.att_controller.pitch_pid.kd = 0;
    ctrl_bus.att_controller.yaw_pid.init();
    ctrl_bus.att_controller.yaw_pid.isIncrement = true;
    ctrl_bus.att_controller.yaw_pid.vcmax = Yaw_Rate_Max;
    ctrl_bus.att_controller.yaw_pid.vcmin = -Yaw_Rate_Max;
    ctrl_bus.att_controller.yaw_pid.kp = 0.03;
    ctrl_bus.att_controller.yaw_pid.ki = 0.0002;
    ctrl_bus.att_controller.yaw_pid.kd = 0.005;

    ctrl_bus.att_controller.roll_rate_pid.init();
    ctrl_bus.att_controller.roll_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.roll_rate_pid.vcmax = Roll_Rate_Max; // 待改动，目前按照手动模式最大值来设置 Roll Thrust Max
    ctrl_bus.att_controller.roll_rate_pid.vcmin = -Roll_Rate_Max;
    ctrl_bus.att_controller.roll_rate_pid.kp = 0;
    ctrl_bus.att_controller.roll_rate_pid.ki = 0;
    ctrl_bus.att_controller.roll_rate_pid.kd = 0;
    ctrl_bus.att_controller.pitch_rate_pid.init();
    ctrl_bus.att_controller.pitch_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.pitch_rate_pid.vcmax = Pitch_Rate_Max; // 待改动，目前按照手动模式最大值来设置 Pitch Thrust Max
    ctrl_bus.att_controller.pitch_rate_pid.vcmin = -Pitch_Rate_Max;
    ctrl_bus.att_controller.pitch_rate_pid.kp = 0.0001;
    ctrl_bus.att_controller.pitch_rate_pid.ki = 0;
    ctrl_bus.att_controller.pitch_rate_pid.kd = 0.00065;
    ctrl_bus.att_controller.yaw_rate_pid.init();
    ctrl_bus.att_controller.yaw_rate_pid.isIncrement = true;
    ctrl_bus.att_controller.yaw_rate_pid.vcmax = Yaw_Rate_Max; // 待改动，目前按照手动模式最大值来设置 Yaw Thrust Max
    ctrl_bus.att_controller.yaw_rate_pid.vcmin = -Yaw_Rate_Max;
    ctrl_bus.att_controller.yaw_rate_pid.kp = -0.04;
    ctrl_bus.att_controller.yaw_rate_pid.ki = 0;
    ctrl_bus.att_controller.yaw_rate_pid.kd = 0;

    // 初始化深度控制器PID
    ctrl_bus.depth_controller.depth_pid.init();
    ctrl_bus.depth_controller.depth_pid.isIncrement = true;
    ctrl_bus.depth_controller.depth_pid.vcmax = Speed_z_Max;
    ctrl_bus.depth_controller.depth_pid.vcmin = -Speed_z_Max;
    ctrl_bus.depth_controller.depth_pid.kp = 1.2;
    ctrl_bus.depth_controller.depth_pid.ki = 0.006;
    ctrl_bus.depth_controller.depth_pid.kd = 0.5;
    ctrl_bus.depth_controller.depth_vel_pid.init();
    ctrl_bus.depth_controller.depth_vel_pid.isIncrement = true;
    ctrl_bus.depth_controller.depth_vel_pid.vcmax = Speed_z_Max; // 待改动，目前按照手动模式最大值来设置
    ctrl_bus.depth_controller.depth_vel_pid.vcmin = -Speed_z_Max;
    ctrl_bus.depth_controller.depth_vel_pid.kp = 0.7;
    ctrl_bus.depth_controller.depth_vel_pid.ki = 0;
    ctrl_bus.depth_controller.depth_vel_pid.kd = 0;

    // 初始化高度控制器PID
    ctrl_bus.alt_controller.alt_pid.init();
    ctrl_bus.alt_controller.alt_pid.isIncrement = true;
    ctrl_bus.alt_controller.alt_pid.vcmax = Speed_z_Max;
    ctrl_bus.alt_controller.alt_pid.vcmin = -Speed_z_Max;
    ctrl_bus.alt_controller.alt_pid.kp = 1.2;
    ctrl_bus.alt_controller.alt_pid.ki = 0.006;
    ctrl_bus.alt_controller.alt_pid.kd = 0.5;
    ctrl_bus.alt_controller.alt_vel_pid.init();
    ctrl_bus.alt_controller.alt_vel_pid.isIncrement = true;
    ctrl_bus.alt_controller.alt_vel_pid.vcmax = Speed_z_Max; // 待改动，目前按照手动模式最大值来设置
    ctrl_bus.alt_controller.alt_vel_pid.vcmin = -Speed_z_Max;
    ctrl_bus.alt_controller.alt_vel_pid.kp = 0.7;
    ctrl_bus.alt_controller.alt_vel_pid.ki = 0;
    ctrl_bus.alt_controller.alt_vel_pid.kd = 0;


    // 初始化输出
    ctrl_bus.alt_controller.throttle_thrust  = ctrl_bus.depth_controller.throttle_thrust = ctrl_bus.att_controller.roll_thrust = ctrl_bus.att_controller.pitch_thrust = ctrl_bus.att_controller.yaw_thrust = 0.0f;
}

void control_interface_step(uint32_t timestamp)
{
    ctrl_bus.ctrl_ins_bus = &ins_bus;
    ctrl_bus.ctrl_fms_bus = &fms_bus;

    control_step();
}

void Clear_pid_err(void)
{
    ctrl_bus.att_controller.pitch_pid.err = 0;
    ctrl_bus.att_controller.pitch_pid.lastErr = 0;
    ctrl_bus.att_controller.pitch_pid.last2Err = 0;

    ctrl_bus.att_controller.roll_pid.err = 0;
    ctrl_bus.att_controller.roll_pid.lastErr = 0;
    ctrl_bus.att_controller.roll_pid.last2Err = 0;

    ctrl_bus.att_controller.yaw_pid.err = 0;
    ctrl_bus.att_controller.yaw_pid.lastErr = 0;
    ctrl_bus.att_controller.yaw_pid.last2Err = 0;

    ctrl_bus.att_controller.roll_rate_pid.err = 0;
    ctrl_bus.att_controller.roll_rate_pid.lastErr = 0;
    ctrl_bus.att_controller.roll_rate_pid.last2Err = 0;

    ctrl_bus.att_controller.pitch_rate_pid.err = 0;
    ctrl_bus.att_controller.pitch_rate_pid.lastErr = 0;
    ctrl_bus.att_controller.pitch_rate_pid.last2Err = 0;

    ctrl_bus.att_controller.yaw_rate_pid.err = 0;
    ctrl_bus.att_controller.yaw_rate_pid.lastErr = 0;
    ctrl_bus.att_controller.yaw_rate_pid.last2Err = 0;

    ctrl_bus.depth_controller.depth_pid.err = 0;
    ctrl_bus.depth_controller.depth_pid.lastErr = 0;
    ctrl_bus.depth_controller.depth_pid.last2Err = 0;

    ctrl_bus.depth_controller.depth_vel_pid.err = 0;
    ctrl_bus.depth_controller.depth_vel_pid.lastErr = 0;
    ctrl_bus.depth_controller.depth_vel_pid.last2Err = 0;
}

static void Manual_control()
{
    // ctrl_bus.depth_controller.depth_pid.ref=ctrl_bus.depth_controller.depth_pid.fb;
    // ctrl_bus.depth_controller.depth_vel_pid.ref = ctrl_bus.depth_controller.depth_pid.calc(ctrl_bus.depth_controller.depth_pid.fb);
    // ctrl_bus.depth_controller.throttle_thrust = ctrl_bus.depth_controller.depth_vel_pid.calc(ctrl_bus.depth_controller.depth_vel_pid.fb);

    // ctrl_bus.depth_controller.forward_thrust = ctrl_bus.ctrl_fms_bus->fms_out.u_cmd;
    // ctrl_bus.depth_controller.lateral_thrust = ctrl_bus.ctrl_fms_bus->fms_out.v_cmd;
    ctrl_bus.alt_controller.throttle_thrust = ctrl_bus.ctrl_fms_bus->fms_out.w_cmd;
    ctrl_bus.att_controller.roll_thrust = ctrl_bus.ctrl_fms_bus->fms_out.p_cmd;
    ctrl_bus.att_controller.pitch_thrust = ctrl_bus.ctrl_fms_bus->fms_out.q_cmd;
    ctrl_bus.att_controller.yaw_thrust = ctrl_bus.ctrl_fms_bus->fms_out.r_cmd;

    ctrl_bus.alt_controller.alt_pid.vc = ctrl_bus.ctrl_ins_bus->alt->altvel;
    ctrl_bus.alt_controller.alt_vel_pid.vc = ctrl_bus.alt_controller.throttle_thrust;

    // ctrl_bus.att_controller.yaw_rate_pid.vc = ctrl_bus.att_controller.yaw_thrust;
    // ctrl_bus.att_controller.yaw_pid.vc = ctrl_bus.ctrl_ins_bus->imu->gyr_z;
    ctrl_bus.att_controller.yaw_pid.vc = ctrl_bus.att_controller.yaw_thrust;
    ctrl_bus.att_controller.pitch_rate_pid.vc = ctrl_bus.att_controller.pitch_thrust;
    ctrl_bus.att_controller.pitch_pid.vc = ctrl_bus.ctrl_ins_bus->imu->gyr_y;
    ctrl_bus.att_controller.roll_rate_pid.vc = ctrl_bus.att_controller.roll_thrust;
    ctrl_bus.att_controller.roll_pid.vc = ctrl_bus.ctrl_ins_bus->imu->gyr_x;
}

static void roll_pitch_control(float set_roll, float set_pitch)
{
    ctrl_bus.att_controller.roll_pid.ref = set_roll;
    ctrl_bus.att_controller.pitch_pid.ref = set_pitch;
    ctrl_bus.att_controller.roll_rate_pid.ref = ctrl_bus.att_controller.roll_pid.calc(ctrl_bus.att_controller.roll_pid.fb);
    ctrl_bus.att_controller.pitch_rate_pid.ref = ctrl_bus.att_controller.pitch_pid.calc(ctrl_bus.att_controller.pitch_pid.fb);
    // 串级
    ctrl_bus.att_controller.roll_thrust = ctrl_bus.att_controller.roll_rate_pid.calc(ctrl_bus.att_controller.roll_rate_pid.fb);
    ctrl_bus.att_controller.pitch_thrust = ctrl_bus.att_controller.pitch_rate_pid.calc(ctrl_bus.att_controller.pitch_rate_pid.fb);
    // 单回路
    // ctrl_bus.att_controller.roll_thrust = ctrl_bus.att_controller.roll_pid.calc(ctrl_bus.att_controller.roll_pid.fb);
    // ctrl_bus.att_controller.pitch_thrust = ctrl_bus.att_controller.pitch_pid.calc(ctrl_bus.att_controller.pitch_pid.fb);
}

static void yaw_rate_control(float set_yaw_rate)
{
    ctrl_bus.att_controller.yaw_rate_pid.ref = set_yaw_rate;
    ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_rate_pid.calc(ctrl_bus.att_controller.yaw_rate_pid.fb);
}

static void yaw_heading_control(float set_yaw)
{
    ctrl_bus.att_controller.yaw_pid.ref = set_yaw;
    if (((ctrl_bus.att_controller.yaw_pid.fb - set_yaw) < M_PI) && ((ctrl_bus.att_controller.yaw_pid.fb - set_yaw) > -M_PI))
    {
        // ctrl_bus.att_controller.yaw_rate_pid.ref = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb);
        // ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_rate_pid.calc(ctrl_bus.att_controller.yaw_rate_pid.fb);
        ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb);
    }
    else if ((ctrl_bus.att_controller.yaw_pid.fb - set_yaw) >= M_PI)
    {
        //     ctrl_bus.att_controller.yaw_rate_pid.ref = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb - 2 * M_PI);
        //     ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_rate_pid.calc(ctrl_bus.att_controller.yaw_rate_pid.fb);
        ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb - 2 * M_PI) ;
    }
    else
    {
        //     ctrl_bus.att_controller.yaw_rate_pid.ref = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb + 2 * M_PI);
        //     ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_rate_pid.calc(ctrl_bus.att_controller.yaw_rate_pid.fb);
        ctrl_bus.att_controller.yaw_thrust = ctrl_bus.att_controller.yaw_pid.calc(ctrl_bus.att_controller.yaw_pid.fb + 2 * M_PI) ;
    }
}

static void xy_control(float set_hor[2] ,float cur_yaw)
{
   
    float cur_pos[2] = {ctrl_bus.ctrl_ins_bus->gps->global_position[0], ctrl_bus.ctrl_ins_bus->gps->global_position[1]};
    float cur_v[2] = {ctrl_bus.ctrl_ins_bus->gps->global_vel[0], ctrl_bus.ctrl_ins_bus->gps->global_vel[1]};

    // 位置误差
    float error_pos[2] = {set_hor[0] - cur_pos[0], set_hor[1] - cur_pos[1]};

    float K_xy_p = 0.5f;

    // 外环：位置误差 -> 期望速度（期望速度是位置控制的输出）
    float desired_vel[2] = {error_pos[0] * K_xy_p, error_pos[1] * K_xy_p};

    // 内环：速度控制
    static float prev_error_vel[2] = {0.0f, 0.0f};  // 速度误差的上一时刻值
    static float integral_vel[2] = {0.0f, 0.0f};    // 速度误差的积分部分

    float error_vel[2] = {desired_vel[0] - cur_v[0], desired_vel[1] - cur_v[1]};

    float K_xyv_p = 0.5f;
    float K_xyv_i = 0.5f;
    float K_xyv_d = 0.5f;

    // 积分部分
    integral_vel[0] += error_vel[0];
    integral_vel[1] += error_vel[1];

    // 微分部分
    float diff_vel[2] = {error_vel[0] - prev_error_vel[0], error_vel[1] - prev_error_vel[1]};

    // 速度控制器的输出：期望加速度
    float desired_acc[2];
    desired_acc[0] = K_xyv_p * error_vel[0] + K_xyv_i * integral_vel[0] + K_xyv_d * diff_vel[0];
    desired_acc[1] = K_xyv_p * error_vel[1] + K_xyv_i * integral_vel[1] + K_xyv_d * diff_vel[1];
    float desired_roll = 1/CONSTANT_G *(sin(cur_yaw)*desired_acc[0]-cos(cur_yaw)*desired_acc[1] );
    float desired_pitch = 1/CONSTANT_G *(cos(cur_yaw)*desired_acc[0]+ sin(cur_yaw)*desired_acc[1] );

    prev_error_vel[0] = error_vel[0];
    prev_error_vel[1] = error_vel[1];
    ctrl_bus.att_controller.roll_pid.ref = desired_roll;
    ctrl_bus.att_controller.pitch_pid.ref = desired_pitch;
}

static void alt_control(float set_alt)
{
    float cur_alt = ctrl_bus.ctrl_ins_bus->gps->global_position[2]; //气压计 激光测距 GPS ，在FMS要对数据做处理
    float cur_alt_vel = ctrl_bus.ctrl_ins_bus->gps->global_vel[2];
    float error_alt = set_alt - cur_alt;
    float K_alt_p = 0.5f;
    float desired_vel = error_alt * K_alt_p;
    static float prev_error_vel = 0.0f;  // 速度误差的上一时刻值
    static float integral_vel = 0.0f;    // 速度误差的积分部分

    float error_alt_vel = cur_alt_vel - desired_vel;

    float K_alt_p = 0.5f;
    float K_alt_i = 0.5f;
    float K_alt_d = 0.5f;

    integral_vel += error_alt_vel;
    float diff_vel = error_alt_vel - prev_error_vel;

    float desired_acc;
    desired_acc = K_alt_p * error_alt_vel + K_alt_i*integral_vel+K_alt_d*diff_vel;
    float desired_T = WEIGHT*(CONSTANT_G + desired_acc);

    prev_error_vel = error_alt_vel;
    ctrl_bus.alt_controller.throttle_thrust = desired_T;
}

static void depth_control(float set_depth)
{
    ctrl_bus.depth_controller.depth_pid.ref = set_depth;

    ctrl_bus.depth_controller.depth_vel_pid.ref = ctrl_bus.depth_controller.depth_pid.calc(ctrl_bus.depth_controller.depth_pid.fb);
    ctrl_bus.depth_controller.throttle_thrust = ctrl_bus.depth_controller.depth_vel_pid.calc(ctrl_bus.depth_controller.depth_vel_pid.fb);
}

static void depth_rate_control(float set_depth_vel)
{
    ctrl_bus.depth_controller.depth_vel_pid.ref = set_depth_vel;
    ctrl_bus.depth_controller.throttle_thrust = ctrl_bus.depth_controller.depth_vel_pid.calc(ctrl_bus.depth_controller.depth_vel_pid.fb);
}

static void alt_control(float set_alt)
{
    ctrl_bus.alt_controller.alt_pid.ref = set_alt;

    ctrl_bus.alt_controller.alt_vel_pid.ref = ctrl_bus.alt_controller.alt_pid.calc(ctrl_bus.alt_controller.alt_pid.fb);
    ctrl_bus.alt_controller.throttle_thrust = ctrl_bus.alt_controller.alt_vel_pid.calc(ctrl_bus.alt_controller.alt_vel_pid.fb);
}

static void alt_rate_control(float set_alt_vel)
{
    ctrl_bus.alt_controller.alt_vel_pid.ref = set_alt_vel;
    ctrl_bus.alt_controller.throttle_thrust = ctrl_bus.alt_controller.alt_vel_pid.calc(ctrl_bus.alt_controller.alt_vel_pid.fb);
}
// static void forward_lateral_control(float set_forward, float set_lateral)
// {
//     ctrl_bus.depth_controller.forward_thrust = set_forward;
//     ctrl_bus.depth_controller.lateral_thrust = set_lateral;
// }

// static void position_control(float forward, float backward, float left, float right)
// {
// }

// static void thrust_max_init()
// {
//     // 可能还需要和控制器输出一起转换，目前将速度的最大值作为推力的最大值
//     float thrust_max_vertical = abs(Roll_Rate_Max * motor1_factor[0]) +
//                                 abs(Pitch_Rate_Max * motor1_factor[1]) +
//                                 abs(Yaw_Rate_Max * motor1_factor[2]) +
//                                 abs(Speed_z_Max * motor1_factor[3]) +
//                                 abs(Speed_x_Max * motor1_factor[4]) +
//                                 abs(Speed_y_Max * motor1_factor[5]);

//     float thrust_max_horizontal = abs(Roll_Rate_Max * motor5_factor[0]) +
//                                   abs(Pitch_Rate_Max * motor5_factor[1]) +
//                                   abs(Yaw_Rate_Max * motor5_factor[2]) +
//                                   abs(Speed_z_Max * motor5_factor[3]) +
//                                   abs(Speed_x_Max * motor5_factor[4]) +
//                                   abs(Speed_y_Max * motor5_factor[5]);

//     thrust_max = MAX(thrust_max_horizontal, thrust_max_vertical);
// }

static void thrust_alloc()
{
    float r = (float)ctrl_bus.att_controller.roll_thrust * 0.707f;
    float p = (float)ctrl_bus.att_controller.pitch_thrust * 0.707f;
    float y = (float)ctrl_bus.att_controller.yaw_thrust;
    float t = (float)ctrl_bus.depth_controller.throttle_thrust;
    motor_thrust[MOTORS_MOT_1] = t + r + p + y;

    motor_thrust[MOTORS_MOT_2] = t + r - p - y; 

    motor_thrust[MOTORS_MOT_3] = t - r - p + y;

    motor_thrust[MOTORS_MOT_4] = t - r + p - y;

    // PWM输出
    thrust_to_pwm();
}
float hover = 0.3;
static void thrust_to_pwm() // 转速和电压近似为线性关系，推力与转速近似为二次方关系，即推力与电压的二次方成正比关系
{
    for (int i = 0; i < MOTORS_MAX_NUM_MOTORS; i++)
    {
        if (motor_thrust[i] >= 0)
        {
            float out = motor_thrust[i]+hover;
            motor_out[i] =  _constrain(out, 0, 1.0);
        }
        else
        {
            float out = motor_thrust[i]- hover;
            motor_out[i] =  -_constrain(out, -1.0, 0);
        }
    }
}

static void thrust_zero()
{
    for (int i = 0; i < MOTORS_MAX_NUM_MOTORS; i++)
    {
        motor_out[i] = 0.0f;
    }
}

static void control_step()
{
    if (ctrl_bus.ctrl_fms_bus->fms_out.status == Arm)
    {
        ctrl_bus.att_controller.roll_pid.fb = ctrl_bus.ctrl_ins_bus->imu->ang_roll;
        ctrl_bus.att_controller.pitch_pid.fb = ctrl_bus.ctrl_ins_bus->imu->ang_pitch;
        ctrl_bus.att_controller.yaw_pid.fb = ctrl_bus.ctrl_ins_bus->imu->ang_yaw;

        ctrl_bus.att_controller.roll_rate_pid.fb = ctrl_bus.ctrl_ins_bus->imu->gyr_x;
        ctrl_bus.att_controller.pitch_rate_pid.fb = ctrl_bus.ctrl_ins_bus->imu->gyr_y;
        ctrl_bus.att_controller.yaw_rate_pid.fb = ctrl_bus.ctrl_ins_bus->imu->gyr_z;

        ctrl_bus.alt_controller.alt_pid.fb = ctrl_bus.ctrl_ins_bus->alt->altitude;
        ctrl_bus.alt_controller.alt_vel_pid.fb = ctrl_bus.ctrl_ins_bus->alt->altvel;

    
        if (ctrl_bus.ctrl_fms_bus->fms_out.mode == Manual)
        {
            Clear_pid_err();
            Manual_control();
            thrust_alloc();
        }
        else if(ctrl_bus.ctrl_fms_bus->fms_out.mode == Stabilize)
        {
            roll_pitch_control(ctrl_bus.ctrl_fms_bus->fms_out.phi_cmd, ctrl_bus.ctrl_fms_bus->fms_out.theta_cmd);

            // yaw目前是单级PID控制
            yaw_rate_control(ctrl_bus.ctrl_fms_bus->fms_out.r_cmd);

            alt_control(ctrl_bus.ctrl_fms_bus->fms_out.z_cmd);
            // forward_lateral_control(ctrl_bus.ctrl_fms_bus->fms_out.u_cmd, ctrl_bus.ctrl_fms_bus->fms_out.v_cmd);
            thrust_alloc();
        }
        else if(ctrl_bus.ctrl_fms_bus->fms_out.mode == Auto)
        {

            xy_control();
            alt_control();
            yaw_heading_control();
            roll_pitch_control();
            thrust_alloc();
            
        }
    }
    else if (ctrl_bus.ctrl_fms_bus->fms_out.status == Disarm)
    {
        thrust_zero();
    }
}
