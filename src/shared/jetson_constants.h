#pragma once

#include "shared/constants.h"

const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = "51";
const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = "76";
const char* MOTOR_DRIVER_RESET_GPIO                = "168";
const char* DRIVER_CONTROL_ENABLE_GPIO             = "194";

constexpr Platform PLATFORM = Platform::JETSON_NANO;
