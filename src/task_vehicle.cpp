//#include <something.h>
#include "stm32f4xx_hal.h" //不会出现数字类型定义错误
#include "ins/ins_interface.h"
#include "fms/fms_interface.h"
#include "control/control_interface.h"
#include "systim/systim.h"
/**
 * @brief Vehicle task init
 */
void task_vehicle_init(void)
{
    /* init ins model */
    ins_interface_init();

    /* init fms model */
    fms_interface_init();

    /* init controller model */
    control_interface_init();

    /* create event */
    //rtos_event_init(&event_vehicle, "vehicle", FLAG);
    
    /* register a 1kHz timer connected with event */
    //rtos_timer_create(&timer_vehicle, "vehicle", timer_vehicle_update, NULL, 1, TIMER_FLAG_PERIODIC);
    //rtos_timer_start(&timer_vehicle);
}

/*static void timer_vehicle_update(void* parameter)
{
   // rtos_event_send(&event_vehicle, EVENT_VEHICLE_UPDATE);
}*/
// TimeTag ins_timtag ={
//     .tag = 0,
//     .period = 100,
// };

// TimeTag fms_timtag ={
//     .tag = 0,
//     .period = 100,
// };

// TimeTag control_timtag ={
//     .tag = 0,
//     .period = 100,
// };



