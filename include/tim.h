#ifndef tim_h
#define tim_h

#include "stm32f4xx_hal.h"

void pwm_speed_set(uint8_t index, float speed);
float pwm_speed_get(uint8_t index);
void pwm_shutdown(void);
void pwm_stop(int index);
void PWM_duty_set(uint8_t index, float duty);
float PWM_duty_get(uint8_t index);
void tim_init(void);

void input_caputure_pwm_callback(void);
void pwm_caputure_init(void);

#endif