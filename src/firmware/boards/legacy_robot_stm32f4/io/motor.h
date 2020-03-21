#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/**
 * \brief The possible modes of driving a motor.
 */
typedef enum
{
    MOTOR_MODE_COAST    = 0U,
    MOTOR_MODE_BRAKE    = 1U,
    MOTOR_MODE_FORWARD  = 2U,
    MOTOR_MODE_BACKWARD = 3U,
} motor_mode_t;

void motor_init(void);
void motor_shutdown(void);
void motor_set(unsigned int motor_num, motor_mode_t mode, uint8_t pwm_level);
void motor_force_power(void);
void motor_tick(void);

#endif
