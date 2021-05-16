#include "firmware/boards/robot_stm32h7/io/pwm_pin.h"
#include "firmware/app/logger/logger.h"

#include <assert.h>
#include <stdlib.h>

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

void io_pwm_pin_setPwm(PwmPin_t* pwm_pin, float pwm_percentage)
{
    assert(pwm_percentage >= 0 && pwm_percentage <= 1);

    float pulse_max = (float)pwm_pin->timer->Init.Period;
    uint16_t pulse  = (uint16_t)round(pwm_percentage * pulse_max);

    TIM_OC_InitTypeDef timer_config;
    timer_config.OCMode     = TIM_OCMODE_PWM1;
    timer_config.Pulse      = pulse;
    timer_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    timer_config.OCFastMode = TIM_OCFAST_DISABLE;

    TLOG_INFO("Starting timer with pwm %d pulse out of max %d", pulse, (int)pulse_max);

    HAL_TIM_PWM_ConfigChannel(pwm_pin->timer, &timer_config, pwm_pin->timer_channel);
    HAL_TIM_PWM_Start(pwm_pin->timer, pwm_pin->timer_channel);
}
