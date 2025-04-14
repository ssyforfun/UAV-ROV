/*
 * @Author: ADSW514610085 Z514610085@163.com
 * @Date: 2024-09-20 10:55:02
 * @LastEditors: ADSW514610085 Z514610085@163.com
 * @LastEditTime: 2024-12-11 13:10:08
 * @FilePath: \demo3\src\fms\rc_control.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "rc_control.h"
#include "../systim/systim.h"
#include "ppm.h"
#include <math.h>

bool rc_updated;
RC_input_t rc_input;
extern PPM_CLASS ppmInst;

TimeTag rc_interval = {
    .tag = 0,
    .period = 50,
};

void rc_control_init(void)
{
    ppmInst.init(1E-6F);
    rc_updated = false;
}
// 将PPM转成rc的输入信号
void ppm_to_rc(void)
{

    if (ppmInst.speed[2] > -0.95)
    {
        rc_input.ch1 = ppmInst.speed[0];
        rc_input.ch2 = ppmInst.speed[1];
        rc_input.ch3 = ppmInst.speed[2];
        rc_input.ch4 = -ppmInst.speed[3];
        rc_input.ch5 = ppmInst.speed[4];
        rc_input.ch6 = ppmInst.speed[5];
        rc_input.sw1 = round(ppmInst.speed[6]);
        rc_input.sw2 = round(ppmInst.speed[7]);
        rc_input.sw3 = round(ppmInst.speed[8]);
        rc_input.sw4 = round(ppmInst.speed[9]);
    }
    else
    {
        rc_input.ch1 = 0;
        rc_input.ch2 = 0;
        rc_input.ch3 = 0;
        rc_input.ch4 = 0;
        rc_input.ch5 = 0;
        rc_input.ch6 = 0;
        rc_input.sw1 = 1;
        rc_input.sw2 = -1;
        rc_input.sw3 = 0;
        rc_input.sw4 = 1;
    }
}

void pilot_cmd_collect()
{
    rc_updated = false;
    if (check_timetag(&rc_interval))
    {
        
        ppm_to_rc();

        rc_updated = true;
    }
}