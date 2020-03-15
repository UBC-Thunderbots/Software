#include "firmware_new/boards/frankie_v1/io/motor.h"

#include <stdlib.h>

#include "firmware_new/boards/frankie_v1/stm32h7xx_hal_conf.h"

typedef struct Motor
{
    TIM_HandleTypeDef* pwm_timer;
    uint16_t pwm_timer_channel;
    GpioPin_t* reset_pin;
    GpioPin_t* coast_pin;
    GpioPin_t* mode_pin;
    GpioPin_t* direction_pin;
    GpioPin_t* brake_pin;
    GpioPin_t* esf_pin;
} Motor_t;

Motor_t* io_motor_create(TIM_HandleTypeDef* pwm_timer, uint16_t pwm_timer_channel,
                         GpioPin_t* reset_pin, GpioPin_t* coast_pin, GpioPin_t* mode_pin,
                         GpioPin_t* direction_pin, GpioPin_t* brake_pin,
                         GpioPin_t* esf_pin)
{
    Motor_t* motor = (Motor_t*)malloc(sizeof(Motor_t));

    motor->pwm_timer         = pwm_timer;
    motor->pwm_timer_channel = pwm_timer_channel;
    motor->reset_pin         = reset_pin;
    motor->coast_pin         = coast_pin;
    motor->mode_pin          = mode_pin;
    motor->direction_pin     = direction_pin;
    motor->brake_pin         = brake_pin;
    motor->esf_pin           = esf_pin;

    io_gpio_pin_setInactive(motor->reset_pin);
    io_gpio_pin_setInactive(motor->coast_pin);
    io_gpio_pin_setActive(motor->mode_pin);
    // TODO: what direction is CW and CCW???
    io_gpio_pin_setActive(motor->direction_pin);
    io_gpio_pin_setInactive(motor->brake_pin);
    // TODO: this should _really_ be active, but we have it inactive for now to get
    //       around the fact that the motor boards have a ton of shorts
    io_gpio_pin_setInactive(motor->esf_pin);

    return motor;
}

void io_motor_destroy(Motor_t* motor)
{
    free(motor);
}

void io_motor_set_direction(Motor_t* motor, MotorDriveDirection direction)
{
    switch (direction)
    {
        case COUNTERCLOCKWISE:
            io_gpio_pin_setActive(motor->direction_pin);
            return;
        case CLOCKWISE:
            io_gpio_pin_setActive(motor->direction_pin);
            return;
    }
}

void io_motor_set_pwm(Motor_t* motor, float pwm_value)
{
    // TODO: assert pwm value in [0,1]
    // TODO: not the appropriate way to get the max value of a uint16_t
    // Rescale PWM
    uint16_t scaled_pwm = (uint16_t)round(pwm_value * pow(2.0, 16.0));

    // TODO: better name for this
    TIM_OC_InitTypeDef timer_config;

    timer_config.OCMode     = TIM_OCMODE_PWM1;
    timer_config.Pulse      = scaled_pwm;
    timer_config.OCPolarity = TIM_OCPOLARITY_HIGH;
    timer_config.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(motor->pwm_timer, &timer_config, motor->pwm_timer_channel);
    HAL_TIM_PWM_Start(motor->pwm_timer, motor->pwm_timer_channel);

    // TODO: add check like the following to the above
    //  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    //  {
    //      Error_Handler();
    //  }
}
