#include "software/primitive/chip_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string ChipPrimitive::PRIMITIVE_NAME = "Chip Primitive";

ChipPrimitive::ChipPrimitive(unsigned int robot_id, const Point &chip_origin,
                             const Angle &chip_direction, double chip_distance_meters)
    : robot_id(robot_id),
      chip_origin(chip_origin),
      chip_direction(chip_direction),
      chip_distance_meters(chip_distance_meters)
{
}

std::string ChipPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int ChipPrimitive::getRobotId() const
{
    return robot_id;
}

Point ChipPrimitive::getChipOrigin() const
{
    return chip_origin;
}

Angle ChipPrimitive::getChipDirection() const
{
    return chip_direction;
}

double ChipPrimitive::getChipDistance() const
{
    return chip_distance_meters;
}

void ChipPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool ChipPrimitive::operator==(const ChipPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->chip_origin == other.chip_origin &&
           this->chip_direction == other.chip_direction &&
           this->chip_distance_meters == other.chip_distance_meters;
}

bool ChipPrimitive::operator!=(const ChipPrimitive &other) const
{
    return !((*this) == other);
}
