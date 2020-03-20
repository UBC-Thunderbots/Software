#ifndef PINS_H
#define PINS_H

#include <gpio.h>

extern const gpio_init_pin_t PINS_INIT[4U][16U];

#define PIN_MRF_CS GPIOA, 15U
#define PIN_MRF_WAKE GPIOB, 6U
#define PIN_MRF_NRESET GPIOB, 7U
#define PIN_MRF_INT GPIOC, 12U

#define PIN_ESTOP_SUPPLY GPIOB, 0U

#endif
