#include "servo_control.h"

SERVO_Bus servo_bus;
extern FMS_Bus fms_bus;

void servo_init(void)
{
    servo_bus.servo_fms_bus = &fms_bus;
}

void servo_step()
{   
    float cmd = servo_bus.servo_fms_bus->fms_out.servo_cmd;
    //PWM OUT;
}