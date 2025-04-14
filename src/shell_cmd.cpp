/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-12-03 13:51:09
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-12-18 11:50:42
 * @FilePath: \demo3\src\shell_cmd.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "shell_cmd.h"
#include "math.h"
#include "fms/gcs_control.h"

extern float yaw_desired;
extern UartIMU uartImu;
extern float yawout;
extern float yaw_feedback;
extern float yaw_rate_desired;
float yaw_shell = 0.0f;
extern float depth_desired;
extern float depth_feedback;
extern float depthout;
extern PPM_CLASS ppmInst;
bool gcs_updated = false;
extern GCS_input_t gcs_input;
extern shell_priority_list shell_priority;
extern CTRL_Bus ctrl_bus;
extern float temperature;
extern float pressure;
extern imu_data_t imu_data;
extern mag_data_t mag_data;
extern UartAtkTof uartTof;
extern bool roll_pitch_mode;

extern void control_interface_init(void);

static bool taskdone = 0;

void control_task(void *argument)
{
    char buf[256] = "";
    for (int i = 0; i < 10; i++)
    {
        sprintf(buf,"distance1:%.5f, distance2:%.5f, distance3:%.5f, distance4:%.5f, distance5:%.5f, distance6:%.5f\r\n",uartTof.longestDistance[0],uartTof.longestDistance[1],uartTof.longestDistance[2],uartTof.longestDistance[3],uartTof.longestDistance[4],uartTof.longestDistance[5]);
        //  sprintf(buf, "yaw:%f, pitch:%f, roll:%f,gyr_z:%.2f\r\n", uartImu.imuHandle.zitai[2], uartImu.imuHandle.zitai[1], uartImu.imuHandle.zitai[0], imu_data.gyr_radDs[2]);

        // int len = strlen(buf);
        // sprintf(&buf[len], "mag_x:%.2f, mag_y:%.2f, mag_z:%.2f\r\n", uartImu.imuHandle.magnet[0], uartImu.imuHandle.magnet[1], uartImu.imuHandle.zitai[2]);

        // sprintf(&buf[len], "yaw_desired:%.2f,yawout:%.2f,yaw_feedback:%.2f,yaw_rate_desired:%.2f\r\n", yaw_desired / M_PI * 180.0f, yawout, yaw_feedback / M_PI * 180.0f, yaw_rate_desired);

        int len = strlen(buf);
        sprintf(&buf[len], "depth_desired:%.2f,depth_feedback:%.2f,depthout:%.2f\r\n", depth_desired, depth_feedback, depthout);

        // len = strlen(buf);
        // sprintf(&buf[len], "temperature:%.2f,pressure:%.2f\r\n", temperature, pressure);

        if (FreeRtosShellSend != NULL)
            FreeRtosShellSend((uint8_t *)buf, strlen(buf));

        osDelay(1000);
    }

    taskdone = true;
    osThreadTerminate(NULL);
}
static BaseType_t print_sensor_data(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;
    osThreadId temp = 0;

    osThreadDef(controlTask, control_task, osPriority::osPriorityNormal, 0, 1024);
    temp = osThreadCreate(osThread(controlTask), NULL);

    while (taskdone != true)
    {
        osDelay(10);
    }

    osThreadTerminate(temp);
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("sensor", "sensor:print the sensor data", print_sensor_data, 0);

void test_task(void *argument)
{
    char buf[256] = "";
    for (int i = 0; i < 60; i++)
    {

        sprintf(buf, "yaw:%f, pitch:%f, roll:%f\r\n", uartImu.imuHandle.zitai[2], uartImu.imuHandle.zitai[1], uartImu.imuHandle.zitai[0]);

        int len = strlen(buf);
        sprintf(&buf[len], "yaw_desired:%.2f,yawout:%.2f,yaw_feedback:%.2f,yaw_rate_desired:%.2f\r\n", yaw_desired / M_PI * 180.0f, yawout, yaw_feedback / M_PI * 180.0f, yaw_rate_desired);

        len = strlen(buf);
        sprintf(&buf[len], "depth_desired:%.2f,depth_feedback:%.2f,depthout:%.2f\r\n", depth_desired, depth_feedback, depthout);

        if (FreeRtosShellSend != NULL)
            FreeRtosShellSend((uint8_t *)buf, strlen(buf));

        osDelay(500);
    }

    taskdone = true;
    osThreadTerminate(NULL);
}
static BaseType_t print_sensor_data_test(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;
    osThreadId temp = 0;

    osThreadDef(testTask, test_task, osPriority::osPriorityNormal, 0, 1024);
    temp = osThreadCreate(osThread(testTask), NULL);

    while (taskdone != true)
    {
        osDelay(10);
    }

    osThreadTerminate(temp);
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("sensor_test", "sensor:print the sensor data for test", print_sensor_data_test, 0);

static BaseType_t yaw_degree_set(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 2;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    yaw_shell = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));
    gcs_input.yaw_disired_cmd = yaw_shell / 180.0f * M_PI;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("yaw", "set the yaw degree", yaw_degree_set, 1);

static BaseType_t depth(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 2;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    gcs_input.depth_disired_cmd = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    if (gcs_input.depth_disired_cmd <= ctrl_bus.depth_controller.depth_pid.fb)
    {
        ctrl_bus.depth_controller.depth_pid.kp = 1.2;
        ctrl_bus.depth_controller.depth_pid.ki = 0.009;
        ctrl_bus.depth_controller.depth_pid.kd = 0.5;
        ctrl_bus.depth_controller.depth_vel_pid.kp = 0.7;
        ctrl_bus.depth_controller.depth_vel_pid.ki = 0;
        ctrl_bus.depth_controller.depth_vel_pid.kd = 0;
    }
    else
    {
        ctrl_bus.depth_controller.depth_pid.kp = 0.85;
        ctrl_bus.depth_controller.depth_pid.ki = 0.008;
        ctrl_bus.depth_controller.depth_pid.kd = 0.95;
        ctrl_bus.depth_controller.depth_vel_pid.kp = 2.16;
        ctrl_bus.depth_controller.depth_vel_pid.ki = 0;
        ctrl_bus.depth_controller.depth_vel_pid.kd = 0;
    }

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth", "set the depth", depth, 1);

static BaseType_t gcs(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    shell_priority = FullControl;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("gcs", "use gcs to fullcontrol the ROV", gcs, 0);

static BaseType_t cooperate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    shell_priority = Cooperate;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("cooperate", "use rc and gcs to control the ROV", cooperate, 0);

static BaseType_t rc(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    shell_priority = ShellDisable;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("rc", "use rc to control the ROV", rc, 0);

static BaseType_t gcsarm(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    gcs_input.gcs_input_status = Arm;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("gcsarm", "make the gcs arm", gcsarm, 0);

static BaseType_t gcsdisarm(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    gcs_input.gcs_input_status = Disarm;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("gcsdisarm", "make the gcs disarm", gcsdisarm, 0);

static BaseType_t gcsmanual(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    gcs_input.gcs_input_status = Arm;
    gcs_input.gcs_input_mode = Manual;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("gcsmanual", "make the gcs manual", gcsmanual, 0);

static BaseType_t gcsstabilize(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    gcs_input.gcs_input_status = Arm;
    gcs_input.gcs_input_mode = Stabilize;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("gcsstabilize", "make the gcs stabilize", gcsstabilize, 0);

static BaseType_t move_forward(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 4;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    gcs_input.forward_cmd = -atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));
    osDelay(1000);
    gcs_input.forward_cmd = 0;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("w", "rov move forward", move_forward, 1);

static BaseType_t move_backward(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 4;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    gcs_input.forward_cmd = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));
    osDelay(1000);
    gcs_input.forward_cmd = 0;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("s", "rov move backward", move_backward, 1);

static BaseType_t move_left(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 4;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    gcs_input.lateral_cmd = -atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));
    osDelay(1000);
    gcs_input.lateral_cmd = 0;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("a", "rov move left", move_left, 1);

static BaseType_t move_right(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 4;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    gcs_input.lateral_cmd = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));
    osDelay(1000);
    gcs_input.lateral_cmd = 0;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("d", "rov move right", move_right, 1);

static BaseType_t depth_kp(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_pid.kp = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_kp", "change depth kp", depth_kp, 1);

static BaseType_t depth_ki(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_pid.ki = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_ki", "change depth ki", depth_ki, 1);

static BaseType_t depth_kd(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_pid.kd = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_kd", "change depth kd", depth_kd, 1);

static BaseType_t depth_vel_kp(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_vel_pid.kp = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_vel_kp", "change depth vel kp", depth_vel_kp, 1);

static BaseType_t depth_vel_ki(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_vel_pid.ki = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_vel_ki", "change depth vel ki", depth_vel_ki, 1);

static BaseType_t depth_vel_kd(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    BaseType_t length = 5;
    BaseType_t *lengthp = &length;
    UBaseType_t position = 1;

    ctrl_bus.depth_controller.depth_vel_pid.kd = atof(FreeRTOS_CLIGetParameter(pcCommandString, position, lengthp));

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("depth_vel_kd", "change depth vel kd", depth_vel_kd, 1);

static BaseType_t r_p_c_o(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    roll_pitch_mode=false;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("open_roll_pitch_control", "open roll pitch control", r_p_c_o, 0);

static BaseType_t r_p_c_c(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
    BaseType_t ret = pdTRUE;
    taskdone = false;

    roll_pitch_mode=true;

    taskdone = true;
    ret = pdFALSE;
    return ret;
}
FREERTOS_SHELL_CMD_REGISTER("stop_roll_pitch_control", "stop roll pitch control", r_p_c_c, 0);