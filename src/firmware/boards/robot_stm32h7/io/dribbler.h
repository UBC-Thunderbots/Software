#pragma once

#include <inttypes.h>
#include "firmware/boards/robot_stm32h7/io/allegro_a3931_motor_driver.h"

// TODO (#2081) placeholders
void io_dribbler_init(AllegroA3931MotorDriver_t* dribbler);
void io_dribbler_setSpeed(uint32_t rpm);
void io_dribbler_coast(void);
unsigned int io_dribbler_getTemperature(void);
