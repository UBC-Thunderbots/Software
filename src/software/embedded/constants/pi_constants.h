#pragma once

#include "software/embedded/platform.h"

constexpr int SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = 16;
constexpr int SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = 19;
constexpr int MOTOR_DRIVER_RESET_GPIO                = 12;
constexpr int DRIVER_CONTROL_ENABLE_GPIO             = 22;

constexpr Platform PLATFORM = Platform::RASP_PI;
