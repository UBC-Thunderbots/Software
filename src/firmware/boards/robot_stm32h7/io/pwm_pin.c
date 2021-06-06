#include "firmware/boards/robot_stm32h7/io/pwm_pin.h"

#include <assert.h>
#include <stdlib.h>

#include "firmware/app/logger/logger.h"

typedef struct PwmPin
{
    TIM_HandleTypeDef* timer;
    uint16_t timer_channel;
} PwmPin_t;

PwmPin_t* io_pwm_pin_create(TIM_HandleTypeDef* timer, uint16_t timer_channel)
{
    PwmPin_t* pwm_pin = (PwmPin_t*)malloc(sizeof(PwmPin_t));

    pwm_pin->timer         = timer;
    pwm_pin->timer_channel = timer_channel;

    TIM_OC_InitTypeDef timer_config;
    timer_config.OCMode      = TIM_OCMODE_PWM1;
    timer_config.Pulse       = 0;  // Start with no PWM
    timer_config.OCPolarity  = TIM_OCPOLARITY_HIGH;
    timer_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    timer_config.OCFastMode  = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(pwm_pin->timer, &timer_config, pwm_pin->timer_channel);
    HAL_TIM_PWM_Start(pwm_pin->timer, pwm_pin->timer_channel);

    return pwm_pin;
}

void io_pwm_pin_destroy(PwmPin_t* pwm_pin)
{
    free(pwm_pin);
}

void io_pwm_pin_setPwm(PwmPin_t* pwm_pin, float pwm_percentage)
{
    float pulse_max = (float)pwm_pin->timer->Init.Period;
    uint16_t pulse  = (uint16_t)round(fabs(pwm_percentage) * pulse_max);

    __HAL_TIM_SET_COMPARE(pwm_pin->timer, pwm_pin->timer_channel, pulse);
}
