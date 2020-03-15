#include "firmware_new/boards/frankie_v1/io/motor.h"

#include <stdlib.h>

#include "firmware_new/boards/frankie_v1/stm32h7xx_hal_conf.h"

typedef struct Motor
{
    PwmPin_t* pwm_pin;
    GpioPin_t* reset_pin;
    GpioPin_t* coast_pin;
    GpioPin_t* mode_pin;
    GpioPin_t* direction_pin;
    GpioPin_t* brake_pin;
    GpioPin_t* esf_pin;
} Motor_t;

Motor_t* io_motor_create(PwmPin_t* pwm_pin, GpioPin_t* reset_pin, GpioPin_t* coast_pin,
                         GpioPin_t* mode_pin, GpioPin_t* direction_pin,
                         GpioPin_t* brake_pin, GpioPin_t* esf_pin)
{
    Motor_t* motor = (Motor_t*)malloc(sizeof(Motor_t));

    motor->pwm_pin       = pwm_pin;
    motor->reset_pin     = reset_pin;
    motor->coast_pin     = coast_pin;
    motor->mode_pin      = mode_pin;
    motor->direction_pin = direction_pin;
    motor->brake_pin     = brake_pin;
    motor->esf_pin       = esf_pin;

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
    io_pwm_pin_setPwm(motor->pwm_pin, pwm_value);
}
