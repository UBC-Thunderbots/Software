#pragma once

#include "firmware/boards/frankie_stm32h7/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h"

typedef struct PwmPin PwmPin_t;

/**
 * Create a PWM pin
 *
 * Note that this pin must be setup before being passed to this function. Setting it
 * up in CubeMX should be sufficient.
 * See https://www.waveshare.com/wiki/STM32CubeMX_Tutorial_Series:_PWM for details.
 *
 * @param timer The timer for the PWM pin. This must be kept alive for the lifetime of
 *              the create PWM pin.
 * @param timer_channel The channel within the timer for the PWM pin
 *
 * @return A PWM pin with the current pwm percentage set to zero
 */
PwmPin_t* io_pwm_pin_create(TIM_HandleTypeDef* timer, uint16_t timer_channel);

/**
 * Destroy the given PWM pin
 * @param pwm_pin The PWM pin to destroy
 */
void io_pwm_pin_destroy(PwmPin_t* pwm_pin);

/**
 * Set the pwm percentage on the given PWM pin
 *
 * @param pwm_pin The pwm pin to set the percentage on
 * @param pwm_percentage A value in [0,1] indicating the PWM percentage to apply
 */
void io_pwm_pin_setPwm(PwmPin_t* pwm_pin, float pwm_percentage);
