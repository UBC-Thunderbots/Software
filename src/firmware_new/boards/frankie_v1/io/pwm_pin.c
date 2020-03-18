#include "firmware_new/boards/frankie_v1/io/pwm_pin.h"

#include <stddef.h>

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

    return pwm_pin;
}

void io_pwm_pin_destroy(PwmPin_t* pwm_pin)
{
    free(pwm_pin);
}

void io_pwm_pin_setPwm(PwmPin_t* pwm_pin, float pwm_value)
{
    // TODO: assert pwm value in [0,1]
    // TODO: not the appropriate way to get the max value of a uint16_t
    // Rescale PWM
    uint16_t scaled_pwm = (uint16_t)round(pwm_value * 44);

    // TODO: better name for this
    TIM_OC_InitTypeDef timer_config;

    timer_config.OCMode     = TIM_OCMODE_PWM1;
    timer_config.Pulse      = scaled_pwm;
    timer_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    timer_config.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(pwm_pin->timer, &timer_config, pwm_pin->timer_channel);
    HAL_TIM_PWM_Start(pwm_pin->timer, pwm_pin->timer_channel);
}
