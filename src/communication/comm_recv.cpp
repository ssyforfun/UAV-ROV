
#include "comm_recv.h"
#include "main.h"
#include "signalAcquisition.h"

CommRecv::CommRecv()
{
}

// uint8_t CommRecv::send(uint8_t *cmd, uint8_t *data, uint32_t datalength)
// {
// 	return 0;
// }
// uint8_t CommRecv::send(uint8_t *buf, uint32_t length)
// {
// 	return 0;
// }
// uint8_t CommRecv::send(char *str)
// {
// 	return 0;
// }
// uint8_t CommRecv::send(char cmd, uint8_t *data, uint32_t datalength)
// {
// 	return 0;
// }

void CommRecv::msgAnalysis(uint8_t cmd, uint8_t *data, uint8_t datalen)
{
    _cmd = cmd;
    _dataPtr = data;
    _datalength = datalen;

    // userComm = this; // 有多个子类实例时，这是个比较危险的设置

    switch (cmd)
    {
    case TEST_CMD: // connect
        data_connect();
        break;
    case AQUISITION_CMD: // signal acquisition
        data_aquisition();
        break;
    case CONTROLMODE_CMD:
        data_controlmode();
        break;
    case SHELL_CMD:
        for (uint8_t i = 0; i < datalen; i++)
            FreeRTOS_ShellIRQHandle(data[i]);
        break;
    default:
        break;
    }
}

void CommRecv::data_connect()
{
    char buf[256];
    sprintf(buf, "robot system is running\r\n");
    send(buf);
}

void CommRecv::data_controlmode()
{
    uint8_t buf[256];
    uint8_t *data = _dataPtr;
    uint8_t cmd = _cmd;
    float ftmp[10];

    switch (data[0])
    {
    case 'a':
    {
        if (data[1] == 'r')
        {
            buf[0] = 'a';
            buf[1] = 'r';
            sAdcVal adcval = ADC_FB_Get();
            for (int i = 0; i < ADC_NUMBERS; i++)
            {
                ftmp[i] = adcval.val[i];
            }
            float2bytes(ftmp, &buf[2], ADC_NUMBERS);
            send(cmd, buf, 2 + 4 * ADC_NUMBERS);
        }
        break;
    }
    case 'b':
    {
        if (data[1] == 'c')
        {
            bytes2float(&data[2], ftmp, 4);
            buf[0] = 'b';
            buf[1] = 'c';
            for (int i = 0; i < 4; i++)
            {
                ftmp[i] += 2.0f;
            }
            float2bytes(ftmp, &buf[2], 4);
            send(cmd, buf, 2 + 4 * 4);
        }
        break;
    }
    case 'i': // imu
    {
        if (data[1] == 'z')
        {
            // buf[0] = 'i';
            // buf[1] = 'z';
            // ftmp[0] = uartImu.imuHandle.zitai[0];
            // ftmp[1] = uartImu.imuHandle.zitai[1];
            // ftmp[2] = uartImu.imuHandle.zitai[2];
            // float2bytes(ftmp, &buf[2], 3);
            // send(cmd, buf, 2 + 4 * 3);
            sprintf((char *)buf, "imu zitai xyz [%0.1f, %0.1f, %0.1f]\r\n", uartImu.imuHandle.zitai[0], uartImu.imuHandle.zitai[1], uartImu.imuHandle.zitai[2]);
            send((char *)buf);
        }
        break;
    }
    case 'm':
    {
        if (data[1] == 'v') // pwm speed set
        {
            if (data[2] == 's')
            {
                bytes2float(&data[3], ftmp, 9);
                for (int i = 0; i < 9; i++)
                {
                    pwm_speed_set(i, ftmp[i]);
                }
                buf[0] = 'm';
                buf[1] = 'v';
                buf[2] = 's';
                for (int i = 0; i < 9; i++)
                {
                    ftmp[i] = pwm_speed_get(i);
                }
                float2bytes(ftmp, &buf[3], 9);
                send(cmd, buf, 3 + 4 * 9);
            }
            else if (data[2] == 'r')
            {
                buf[0] = 'm';
                buf[1] = 'v';
                buf[2] = 'r';
                for (int i = 0; i < 9; i++)
                {
                    ftmp[i] = pwm_speed_get(i);
                }
                float2bytes(ftmp, &buf[3], 9);
                send(cmd, buf, 3 + 4 * 9);
            }
            else if (data[2] == 't')
            {
                pwm_shutdown();
                buf[0] = 'm';
                buf[1] = 'v';
                buf[2] = 't';
                send(cmd, buf, 3);
            }
        }
        break;
    }
    case 'r': // reset
    {
        sprintf((char *)buf, "system reset now...\r\n");
        send((char *)buf);
        osDelay(100);
        NVIC_SystemReset();
    }
    default:
        break;
    }
}

void CommRecv::data_aquisition()
{
    SignalAquisition::receiveCmd(_dataPtr);
}
