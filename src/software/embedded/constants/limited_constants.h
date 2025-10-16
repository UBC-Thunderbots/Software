#pragma once

#include "software/embedded/platform.h"

// Dummy constants. Under this compilation mode, the motor control code shouldn't execute.
constexpr int SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = 0;
constexpr int SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = 0;
constexpr int MOTOR_DRIVER_RESET_GPIO                = 0;
constexpr int DRIVER_CONTROL_ENABLE_GPIO             = 0;

// Path to the CPU thermal zone temperature file
constexpr const char CPU_TEMP_FILE_PATH[] = "/sys/class/thermal/thermal_zone0/temp";

// Robot diagnostics constants
constexpr double AUTO_CHIP_DISTANCE_DEFAULT_M = 1.5;
constexpr double AUTO_KICK_SPEED_DEFAULT_M_PER_S = 1.5;
constexpr double WHEEL_ROTATION_MAX_SPEED_M_PER_S = 2.0;

constexpr Platform PLATFORM = Platform::LIMITED_BUILD;
