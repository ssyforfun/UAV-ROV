/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 14:15:33
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-09-23 16:04:27
 * @FilePath: \demo3\src\ins\sensor_driver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H
#include "stm32f4xx_hal.h"
typedef struct {
    uint32_t timestamp_ms;
    float gyr_radDs[3];
    float acc_mDs2[3];
} imu_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float mag_gauss[3];
} mag_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float temperature_deg;
    float pressure_pa;
    float depth_m;
    // float dt_ms;
} bar_data_t;

typedef struct {
    uint32_t timestamp_ms;
    float distance_m;
} rnf_data_t;

void sensor_init();
void sensor_collect();
#endif // SENSOR_DRIVER_H