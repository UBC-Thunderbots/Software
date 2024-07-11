#pragma once

#include "shared/constants.h"

const int SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = 16;
const int SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = 19;
const int MOTOR_DRIVER_RESET_GPIO                = 12;
const int DRIVER_CONTROL_ENABLE_GPIO             = 22;

constexpr Platform PLATFORM = Platform::RASP_PI;
