#pragma once

// TODO: block comment here explaining what we're controlling

// TODO: should change "motor" to include the model of the motor driver we're using

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"
#include "firmware_new/boards/frankie_v1/io/pwm_pin.h"
#include "firmware_new/boards/frankie_v1/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h"

// TODO: should prefix enum values with something to avoid conflicts
typedef enum
{
    CLOCKWISE,
    COUNTERCLOCKWISE
} MotorDriveDirection;

typedef struct Motor Motor_t;

// TODO: jdoc here
Motor_t* io_motor_create(PwmPin_t* pwm_pin, GpioPin_t* reset_pin, GpioPin_t* coast_pin,
                         GpioPin_t* mode_pin, GpioPin_t* direction_pin,
                         GpioPin_t* brake_pin, GpioPin_t* esf_pin);

// TODO: jdoc here
void io_motor_destroy(Motor_t* motor);

// TODO: jdoc here
void io_motor_set_direction(Motor_t* motor, MotorDriveDirection direction);

// TODO: jdoc here
void io_motor_set_pwm(Motor_t* motor, float);