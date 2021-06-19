#pragma once

#include "firmware/boards/robot_stm32h7/io/gpio_pin.h"
void io_chicker_init(GpioPin_t* charger);
void io_chicker_tick();
void io_chicker_kick(float speed_m_per_s);
void io_chicker_chip(float distance_m);
void io_chicker_enable_auto_kick(float speed_m_per_s);
void io_chicker_enable_auto_chip(float distance_m);
void io_chicker_disable_auto_kick();
void io_chicker_disable_auto_chip();
