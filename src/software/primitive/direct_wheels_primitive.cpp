#include "software/primitive/direct_wheels_primitive.h"

#include <cstdint>

#include "software/logger/logger.h"

const std::string DirectWheelsPrimitive::PRIMITIVE_NAME = "Direct Wheels Primitive";

DirectWheelsPrimitive::DirectWheelsPrimitive(
    unsigned int robot_id, int16_t front_left_wheel_power, int16_t back_left_wheel_power,
    int16_t front_right_wheel_power, int16_t back_right_wheel_power, double dribbler_rpm)
    : robot_id(robot_id),
      front_left_wheel_power(front_left_wheel_power),
      back_left_wheel_power(back_left_wheel_power),
      front_right_wheel_power(front_right_wheel_power),
      back_right_wheel_power(back_right_wheel_power),
      dribbler_rpm(dribbler_rpm)
{
    // Clamp wheel power if out of range, log a warning
    int16_t MIN = -255;
    int16_t MAX = 255;

    if (abs(front_left_wheel_power) > 255)
    {
        this->front_left_wheel_power = std::clamp(front_left_wheel_power, MIN, MAX);
        LOG(WARNING) << "direct wheels: front left wheel power out of bounds";
    }
    if (abs(back_left_wheel_power) > 255)
    {
        this->back_left_wheel_power = std::clamp(back_left_wheel_power, MIN, MAX);
        LOG(WARNING) << "direct wheels: back left wheel power out of bounds";
    }
    if (abs(front_right_wheel_power) > 255)
    {
        this->front_right_wheel_power = std::clamp(front_right_wheel_power, MIN, MAX);
        LOG(WARNING) << "direct wheels: front right wheel power out of bounds";
    }
    if (abs(back_right_wheel_power) > 255)
    {
        this->back_right_wheel_power = std::clamp(back_right_wheel_power, MIN, MAX);
        LOG(WARNING) << "direct wheels: back right wheel power out of bounds";
    }
}

std::string DirectWheelsPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int DirectWheelsPrimitive::getRobotId() const
{
    return robot_id;
}

int16_t DirectWheelsPrimitive::getWheel0Power() const
{
    return front_left_wheel_power;
}

int16_t DirectWheelsPrimitive::getWheel1Power() const
{
    return back_left_wheel_power;
}

int16_t DirectWheelsPrimitive::getWheel2Power() const
{
    return front_right_wheel_power;
}

int16_t DirectWheelsPrimitive::getWheel3Power() const
{
    return back_right_wheel_power;
}

double DirectWheelsPrimitive::getDribblerRPM() const
{
    return dribbler_rpm;
}

void DirectWheelsPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DirectWheelsPrimitive::operator==(const DirectWheelsPrimitive &other) const
{
    return this->robot_id == other.robot_id &&
           this->front_left_wheel_power == other.front_left_wheel_power &&
           this->back_left_wheel_power == other.back_left_wheel_power &&
           this->front_right_wheel_power == other.front_right_wheel_power &&
           this->back_right_wheel_power == other.back_right_wheel_power &&
           this->dribbler_rpm == other.dribbler_rpm;
}

bool DirectWheelsPrimitive::operator!=(const DirectWheelsPrimitive &other) const
{
    return !((*this) == other);
}
