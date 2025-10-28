#pragma once

#include <array>

#include "software/util/make_enum/make_enum.hpp"

MAKE_ENUM(MotorIndex, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT, DRIBBLER);

constexpr std::array<MotorIndex, 4> driveMotors()
{
    return {
        MotorIndex::FRONT_LEFT,
        MotorIndex::FRONT_RIGHT,
        MotorIndex::BACK_LEFT,
        MotorIndex::BACK_RIGHT,
    };
};
