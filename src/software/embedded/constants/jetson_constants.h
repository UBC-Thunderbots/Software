#pragma once

#include "software/embedded/platform.h"

constexpr const char SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO[] = "51";
constexpr const char SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO[] = "76";
constexpr const char MOTOR_DRIVER_RESET_GPIO[]                = "168";
constexpr const char DRIVER_CONTROL_ENABLE_GPIO[]             = "194";

// Path to the CPU thermal zone temperature file
constexpr const char CPU_TEMP_FILE_PATH[] = "/sys/class/thermal/thermal_zone1/temp";

// Robot diagnostics constants
constexpr double AUTO_CHIP_DISTANCE_DEFAULT_M = 1.5;
constexpr double AUTO_KICK_SPEED_DEFAULT_M_PER_S = 1.5;
constexpr double WHEEL_ROTATION_MAX_SPEED_M_PER_S = 2.0;

constexpr Platform PLATFORM = Platform::JETSON_NANO;
