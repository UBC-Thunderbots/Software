#include "firmware/boards/robot_stm32h7/io/allegro_a3931_motor_driver.h"

#include <stdlib.h>

typedef struct AllegroA3931MotorDriver
{
    PwmPin_t* pwm_pin;
    GpioPin_t* reset_pin;
    GpioPin_t* mode_pin;
    GpioPin_t* direction_pin;
} AllegroA3931MotorDriver_t;

AllegroA3931MotorDriver_t* io_allegro_a3931_motor_driver_create(PwmPin_t* pwm_pin,
                                                                GpioPin_t* reset_pin,
                                                                GpioPin_t* mode_pin,
                                                                GpioPin_t* direction_pin)
{
    AllegroA3931MotorDriver_t* motor_driver =
        (AllegroA3931MotorDriver_t*)malloc(sizeof(AllegroA3931MotorDriver_t));

    motor_driver->pwm_pin       = pwm_pin;
    motor_driver->reset_pin     = reset_pin;
    motor_driver->mode_pin      = mode_pin;
    motor_driver->direction_pin = direction_pin;

    io_gpio_pin_setActive(motor_driver->reset_pin);
    io_gpio_pin_setActive(motor_driver->mode_pin);
    io_gpio_pin_setActive(motor_driver->direction_pin);

    io_allegro_a3931_motor_setPwmPercentage(motor_driver, 0);

    return motor_driver;
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
    io_pwm_pin_updatePwm(motor_driver->pwm_pin, pwm_percentage);
}
