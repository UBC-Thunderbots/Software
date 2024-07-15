#pragma once

#include "software/embedded/platform.h"

constexpr const char SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO[] = "51";
constexpr const char SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO[] = "76";
constexpr const char MOTOR_DRIVER_RESET_GPIO[]                = "168";
constexpr const char DRIVER_CONTROL_ENABLE_GPIO[]             = "194";

constexpr Platform PLATFORM = Platform::JETSON_NANO;
