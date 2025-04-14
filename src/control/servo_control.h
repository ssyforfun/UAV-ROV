#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"
#include "../fms/fms_interface.h"
typedef struct
{
  FMS_Bus *servo_fms_bus;

} SERVO_Bus;

void servo_init(void);

void servo_step();

#endif