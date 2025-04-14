/************************************************************
 * FileName:        Enable Management.cpp
 * Description:     所有开关的使能控制
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-07-12
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include "setup.h"
#include "main.h"
#include "signalAcquisition.h"
#include "stm32f4xx_hal.h"
#include "ins/ins_interface.h"
#include "fms/fms_interface.h"
#include "control/control_interface.h"
#include "control/send_actuator_cmd.h"
#include "control/init_actuator.h"
#include "shell_cmd.h"
#include "stdio.h"
#include "stdlib.h"
// ------------------------------------------------------------------------------------
uint64_t adcTick = 0;
sAdcVal adcVal;
// 如果需要adc中断处理则使用该函数
static void adc_callback(sAdcVal *val)
{
    adcVal = *val;
    adcTick++;
}

// ------------------------------------------------------------------------------------
extern float motor_out[MOTORS_MAX_NUM_MOTORS];
// ------------------------------------------------------------------------------------
//shell命令函数





TimeTag ins_timtag = {
    .tag = 0,
    .period = 100,
};

TimeTag fms_timtag = {
    .tag = 0,
    .period = 100,
};

TimeTag control_timtag = {
    .tag = 0,
    .period = 100,
};

void vThreadSetup(void *pvParameters)
{
    // uint8_t state = 0;
    char info[256];
    UNUSED(info);

    delay(10); // wait for uart ready
    usb_init();

    adc_received_callback = adc_callback;
    ADC_init();

    can_init();

    uart_init();

    // 建立I2C任务，在I2C任务里进行 验证
    osThreadDef(i2ctask, vThreadI2c, osPriority::osPriorityLow, 0, 1024);
    osThreadCreate(osThread(i2ctask), NULL);

    tim_init();
    pwm_caputure_init();

    W25QXX_Init();
    // W25QXX_test_write(info);

    SignalAquisition::init((CommunicationProtocol **)&userComm);
    osThreadDef(dataAquisition, vThreadDataAquisition, osPriority::osPriorityHigh, 0, 512);
    dataAquisition_thread_id = osThreadCreate(osThread(dataAquisition), NULL);
    delay(10);

    // sprintf(info, "starting acquisition system...\n");
    // userComm->send(info);
    // vTaskDelay(10);

    // pwm_shutdown();

    // sampler.powerEnSet(10, 1); // 52V 电源常开

    // 初始化
    init_actuator();

    task_vehicle_init();

    uint32_t time_start = 0;
    uint32_t time_now;
    uint32_t timestamp;

    while (1)
    {
        time_now = systime_now_ms();
        /* record loop start time */
        if (time_start == 0)
        {
            time_start = time_now;
        }
        /* the model simulation start from 0, so we calcualtet the timestamp relative to start time */
        timestamp = time_now - time_start;

        /* collect sensor data */
        sensor_collect();

        /* collect RC command */
        pilot_cmd_collect();

        /* collect GCS command */
        gcs_cmd_collect();

        /* run INS model every ins_period */
        PERIOD_EXECUTE(&ins_timtag, time_now, ins_interface_step(timestamp));
        /* run FMS model every fms_period */
        PERIOD_EXECUTE(&fms_timtag, time_now, fms_interface_step(timestamp));
        /* run Controller model every control_period */
        PERIOD_EXECUTE(&control_timtag, time_now, control_interface_step(timestamp));

        send_actuator_cmd();

        {
            // adc
            // adc_softTrigger();

            // can
            // can_test(0); // id可以选0或者1

            // 电调
            // float duty = PWM_duty_get(3);
            // sprintf(info, "duty = %0.1f\r\n", duty);
            // usbComm.send(info);

            // 激光测距仪
            // for (int i = 0; i < TOF_NUMBERS; i++)
            // {
            //     sprintf(info, "tof[%d] longest distance = %0.3fm\r\n", i + 1, uartTof.longestDistance[i]);
            //     userComm->send(info);
            //     osDelay(50);
            // }

            // 水压计
            // sprintf(info, "water pressure : %0.1fmbar, temperature : %0.1fC\r\n", pressure, temperature);
            // usbComm.send(info);

            // ppm遥控器
            // if (ppmInst.IsUpdated())
            // {
            //     sprintf(info, "ppm channels=%d, value:[%0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f]\r\n", ppmInst.ppmChannels,
            //             ppmInst.speed[0], ppmInst.speed[1], ppmInst.speed[2], ppmInst.speed[3], ppmInst.speed[4],
            //             ppmInst.speed[5], ppmInst.speed[6], ppmInst.speed[7], ppmInst.speed[8], ppmInst.speed[9]);
            //     usbComm.send(info);
            // }

            // flash
            // W25QXX_test_read(info);
            // usbComm.send(info);

            // imu
            // sprintf(info, "imu zitai : [%0.3f, %0.3f, %0.3f]\r\n", uartImu.imuHandle.zitai[0], uartImu.imuHandle.zitai[1], uartImu.imuHandle.zitai[2]);
            // usbComm.send(info);

            // HAL_GetTick();

            // osDelay(2000);

            // ADC_DMA_callback();
        }
    }

    // 查看任务stack是否溢出
    // sprintf(info, "heap remainsize %d, minimum freesize %d, stack remainsize %d\n",
    //        (int)xPortGetFreeHeapSize(), (int)xPortGetMinimumEverFreeHeapSize(), (int)uxTaskGetStackHighWaterMark(NULL));
    // sprintf(info, "led stack remainsize %d, uart stack remainsize %d, stack remainsize %d\n",
    //        (int)uxTaskGetStackHighWaterMark(led_thread_id), (int)uxTaskGetStackHighWaterMark(uart_thread_id), (int)uxTaskGetStackHighWaterMark(NULL));
}

/*
static void mainTimerCallback(void *arg)
{

    osSignalSet(main_thread_id, 1);
}

static void mainThread(void *arg)
{
    while (1)
    {
        if (osSignalWait(1, osWaitForever).status == osEventSignal)
        {
        }
    }
}*/


// static bool taskdone = 0;

// void control_task(void *argument)
// {
//     char buf[256];
//     static const float ins_angle[3] = {1.2f, 3.4f, 5.6f};
//     static const float force[6] = {2.1f, 3.2f, 4.3f, 5.4f, 6.5f, 7.6f};

//     // osDelay(3000);  //wait for init

//     for (int i = 0; i < 5; i++)
//     {

//         sprintf(buf, "[IMU] YAW:%f, PITCH:%f, ROLL:%f\r\n", ins_angle[0], ins_angle[1], ins_angle[2]);
//         int len = strlen(buf);
//         sprintf(&buf[len], "[Force] 1:%f, 2:%f, 3:%f,4:%f, 5:%f, 6:%f\r\n", force[0], force[1], force[2], force[3], force[4], force[5]);
//         if (FreeRtosShellSend != NULL)
//             FreeRtosShellSend((uint8_t *)buf, strlen(buf));
//         osDelay(1000);
//     }

//     taskdone = true;
//     vTaskDelete(NULL);
// }

// static BaseType_t print_sensor_data(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
// {
//     BaseType_t ret = pdTRUE;
//     taskdone = false;
//     osThreadDef(controlTask, control_task, osPriority::osPriorityNormal, 0, 1024);
//     osThreadCreate(osThread(controlTask), NULL);

//     while (taskdone != true)
//     {
//         osDelay(10);
//     }
//     ret = pdFALSE;
//     return ret;
// }
// FREERTOS_SHELL_CMD_REGISTER("sensor", "sensor:print the sensor data", print_sensor_data, 0);


