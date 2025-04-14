/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-09-23 11:03:51
 * @FilePath: \demo3\src\ins\ins_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef INS_INTERFACE_H
#define INS_INTERFACE_H


#include "sensor_driver.h"
#include "../systim/systim.h"
//#include "uart_imu.h"

typedef struct {
  uint32_t timestamp;
  float gyr_x;
  float gyr_y;
  float gyr_z;
  float acc_x;
  float acc_y;
  float acc_z;
  float ang_roll;
  float ang_pitch;
  float ang_yaw;
  float quat[4];
} IMU_Bus;

typedef struct {
  uint32_t timestamp;
  float mag_x;
  float mag_y;
  float mag_z;
} MAG_Bus;

typedef struct {
  uint32_t timestamp;
  float depth;
  float pressure;
  float temperature;
  float depvel;
} Bar_Bus;

typedef struct {
  uint32_t timestamp;
  float distance;
} Rnf_Bus;

typedef struct {
    uint32_t timestamp;
    const IMU_Bus *imu;
    const MAG_Bus *mag;
    const Bar_Bus *bar;
    const Rnf_Bus *rnf;
} INS_Bus;

void ins_interface_init();
void ins_interface_step(uint32_t timestamp);
void task_vehicle_init(void);

#endif // INS_INTERFACE_H