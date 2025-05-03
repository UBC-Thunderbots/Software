#pragma once

#include "software/util/make_enum/make_enum.hpp"

MAKE_ENUM(MotorBoard, TRINAMIC, STSPIN);

#ifdef TRINAMIC_MOTOR_BOARD
constexpr MotorBoard MOTOR_BOARD = MotorBoard::TRINAMIC;
#elif STSPIN_MOTOR_BOARD
constexpr MotorBoard MOTOR_BOARD = MotorBoard::STSPIN;
#endif
