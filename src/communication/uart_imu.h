
#ifndef _UART_IMU_H
#define _UART_IMU_H

#include "uart_comm.h"

#define IMU_ACC_RANGE (4)
#define IMU_GYPO_RANGE (2000)

struct IMU_TypeOf_Handle
{
    // 姿态角度 x-y-z
    // x-roll  [-180, 180]
    // y-pitch [-90, 90]
    // z-yaw   [-180, 180]
    float zitai[3];

    // 四元数
    // q0-q1-q2-q3
    // [0 1]
    float siyuanshu[4];

    // 陀螺仪 x-y-z
    // 旋转速率 [-2000, 2000] unit °/S
    float gyro[3];

    // 加速度 x-y-z
    // [-4, 4] unit G, 1G=9.8m/s^2
    float acc[3];

    // 磁力 x-y-z
    // [-32768, 32768]
    float magnet[3];

    // 温度 摄氏度
    float temperature;

    // 气压 unit Pa
    float pressure;

    // 海拔高度 unit m
    float altitude;
};

class UartIMU : public UartComm
{
public:
    UartIMU();

public:
    IMU_TypeOf_Handle imuHandle;
    void init(uint8_t *txheader, int sendSize, UART_HandleTypeDef *huart); 

    // ---- 重写 CommunicationProtocol 的 dataAnalysis --------
protected:
    virtual void dataAnalysis();

    // ----  接收数据的分析 --------------------------------------------
public:
    void data_zitai(void);
    void data_siyuanshu(void);
    void data_gyro_acc(void);
    void data_magnet(void);
    void data_pressure(void);

protected:
    enum IMU_COMMAND
    {
        IMUCMD_SAVE = 0,
        ZITAI_CMD = 0x01,
        SIYUANSHU_CMD = 0x02,
        GYRO_ACC_CMD = 0x03,
        MAGNET_CMD = 0x04,
        PRESSURE_CMD = 0x05,
        PORT_STATUS_CMD = 0x06,
        IMUCMD_BAUD = 0x07,
        IMUCMD_RETURNSET = 0x88,
        IMUCMD_RETURNRATE = 0x0A,
        IMUCMD_ALG = 0x0B,
        IMUCMD_RESET = 0x7F,
        IMUCMD_GYRO_RANGE = 0x83,
        IMUCMD_ACC_RANGE = 0x84,
    };

    // ------------ imu设置 --------------------
public:
    void imu_baud_set(uint8_t baudset);
    void imu_returnrate_set(uint8_t returnrate);
    void imu_returncontext_set(uint8_t returncontext);
    void imu_alg_set(uint8_t alg);
    void imu_save(void);
    void imu_gyro_range_set(int range);
    void imu_acc_range_set(int range);
    void imu_led_set(bool on);
};

extern UartIMU uartImu;

#endif
