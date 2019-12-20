#ifndef DRIBBLER_H
#define DRIBBLER_H

#include <stdbool.h>

#include "util/log.h"

/**
 * \ingroup DRIBBLER
 *
 * \brief The frequency at which \ref dribbler_tick should be invoked.
 */
#define DRIBBLER_TICK_HZ 25U

void dribbler_coast(void);
void dribbler_set_speed(uint32_t desired_rpm);
void dribbler_tick(log_record_t *record);
bool dribbler_hot(void);
unsigned int dribbler_temperature(void);
uint8_t dribbler_calc_new_pwm(int32_t current_speed, int32_t desired_speed,
                              uint8_t old_dribbler_pwm);
int32_t hall_speed_to_rpm(int32_t hall_speed);

#endif
