#include "firmware_new/boards/frankie_v1/io/allegro_a3931_motor_driver.h"

#include <stdlib.h>

typedef struct AllegroA3931MotorDriver
{
    PwmPin_t* pwm_pin;
    GpioPin_t* reset_pin;
    GpioPin_t* coast_pin;
    GpioPin_t* mode_pin;
    GpioPin_t* direction_pin;
    GpioPin_t* brake_pin;
    GpioPin_t* esf_pin;
} AllegroA3931MotorDriver_t;

AllegroA3931MotorDriver_t* io_allegro_a3931_motor_driver_create(
    PwmPin_t* pwm_pin, GpioPin_t* reset_pin, GpioPin_t* coast_pin, GpioPin_t* mode_pin,
    GpioPin_t* direction_pin, GpioPin_t* brake_pin, GpioPin_t* esf_pin)
{
    AllegroA3931MotorDriver_t* motor =
        (AllegroA3931MotorDriver_t*)malloc(sizeof(AllegroA3931MotorDriver_t));

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

void io_allegro_a3931_motor_driver_destroy(AllegroA3931MotorDriver_t* motor_driver)
{
    free(motor_driver);
}

void io_allegro_a3931_motor_driver_setDirection(
    AllegroA3931MotorDriver_t* motor_driver,
    AllegroA3931MotorDriverDriveDirection direction)
{
    switch (direction)
    {
        case COUNTERCLOCKWISE:
            io_gpio_pin_setActive(motor_driver->direction_pin);
            return;
        case CLOCKWISE:
            io_gpio_pin_setInactive(motor_driver->direction_pin);
            return;
    }
}

void io_allegro_a3931_motor_setPwmPercentage(AllegroA3931MotorDriver_t* motor_driver,
                                             float pwm_percentage)
{
    io_pwm_pin_setPwm(motor_driver->pwm_pin, pwm_percentage);
}
