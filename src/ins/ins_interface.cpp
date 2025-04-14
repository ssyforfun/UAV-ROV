/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-11-18 15:41:10
 * @FilePath: \demo3\src\ins\ins_interface.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ins_interface.h"
#include "math.h"

#include "../communication/uart_imu.h"
#include "../communication/uart_init.h"

extern imu_data_t imu_data;
extern mag_data_t mag_data;
extern bar_data_t bar_data;
extern rnf_data_t rnf_data;

IMU_Bus imu_bus;
MAG_Bus mag_bus;
Bar_Bus bar_bus;
Rnf_Bus rnf_bus;
INS_Bus ins_bus;

static bool flag = false;
static float depth_base;

TimeTag ins_interval = {
    .tag = 0,
    .period = 100,
};

void ins_interface_init()
{

    sensor_init();
}

uint32_t last_timestamp;
uint16_t last_depth;

uint8_t imu_flag;
uint8_t mag_flag;
uint8_t bar_flag;
uint8_t rnf_flag;

// update函数是将传感器数据搬运一下
static void imu_update(uint32_t timestamp);

static void mag_update(uint32_t timestamp);

static void bar_update(uint32_t timestamp);

static void rnf_update(uint32_t timestamp);

// 对数据做进一步处理
static void ins_step();

void ins_interface_step(uint32_t timestamp)
{
    imu_update(timestamp);
    mag_update(timestamp);
    bar_update(timestamp);
    rnf_update(timestamp);

    ins_step();

    // ins_bus这个结构体就是另外两个模块拿来用的
    ins_bus.bar = &bar_bus;
    ins_bus.imu = &imu_bus;
    ins_bus.mag = &mag_bus;
    ins_bus.rnf = &rnf_bus;
}

static void imu_update(uint32_t timestamp)
{
    imu_bus.timestamp = timestamp;
    imu_bus.gyr_x = imu_data.gyr_radDs[0];
    imu_bus.gyr_y = imu_data.gyr_radDs[1];
    imu_bus.gyr_z = imu_data.gyr_radDs[2];
    imu_bus.acc_x = imu_data.acc_mDs2[0];
    imu_bus.acc_y = imu_data.acc_mDs2[1];
    imu_bus.acc_z = imu_data.acc_mDs2[2];
    imu_flag = 1;
}

static void mag_update(uint32_t timestamp)
{
    mag_bus.timestamp = timestamp;
    mag_bus.mag_x = mag_data.mag_gauss[0];
    mag_bus.mag_y = mag_data.mag_gauss[1];
    mag_bus.mag_z = mag_data.mag_gauss[2];
    mag_flag = 1;
}

static void bar_update(uint32_t timestamp)
{
    bar_bus.timestamp = timestamp;
    if (!flag)
    {
        bar_bus.depth = bar_data.depth_m;
        depth_base = bar_bus.depth;
        flag = true;
    }
    else
    {
        bar_bus.depth = bar_data.depth_m - depth_base;
    }
    bar_bus.pressure = bar_data.pressure_pa;
    bar_bus.temperature = bar_data.temperature_deg;

    bar_flag = 1;
}

static void rnf_update(uint32_t timestamp)
{
    rnf_bus.timestamp = timestamp;
    rnf_bus.distance = rnf_data.distance_m;
    rnf_flag = 1;
}

static void ins_step()
{
    if (imu_flag)
    {
        imu_flag = 0;
        // 滤波函数得到四元数和姿态角
        imu_bus.ang_roll = uartImu.imuHandle.zitai[0] / (180.0f) * M_PI;
        imu_bus.ang_pitch = uartImu.imuHandle.zitai[1] / (180.0f) * M_PI;
        imu_bus.ang_yaw = uartImu.imuHandle.zitai[2] / (180.0f) * M_PI;
        imu_bus.quat[0] = uartImu.imuHandle.siyuanshu[0];
        imu_bus.quat[1] = uartImu.imuHandle.siyuanshu[1];
        imu_bus.quat[2] = uartImu.imuHandle.siyuanshu[2];
        imu_bus.quat[3] = uartImu.imuHandle.siyuanshu[3];
    }
    if (mag_flag)
    {
        mag_flag = 0;
    }
    if (bar_flag)
    {
        bar_flag = 0;
        uint32_t dt_dep = (bar_bus.timestamp >= last_timestamp) ? (bar_bus.timestamp - last_timestamp) : (0xFFFFFFFF - last_timestamp + bar_bus.timestamp);
        last_timestamp = bar_bus.timestamp;
        uint32_t dep_vel = (bar_bus.depth - last_depth) / dt_dep;
        last_depth = bar_bus.depth;
        bar_bus.depvel = dep_vel;
    }
    if (rnf_flag)
    {
        rnf_flag = 0;
    }
}