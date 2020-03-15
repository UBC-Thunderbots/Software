#pragma once

#include "stm32h7xx_hal.h"

typedef struct PwmPin PwmPin_t;

// TODO: jdoc here
PwmPin_t* io_pwm_pin_create(TIM_HandleTypeDef* timer, uint16_t timer_channel);

// TODO: jdoc here
void io_pwm_pin_destroy(PwmPin_t* pwm_pin);

// TODO: jdoc here
void io_pwm_pin_setPwm(PwmPin_t* pwm_pin, float pwm_value);
