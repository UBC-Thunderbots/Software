#include "ai/primitive/chip_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string ChipPrimitive::PRIMITIVE_NAME = "Chip Primitive";

ChipPrimitive::ChipPrimitive(unsigned int robot_id, const Point &chip_origin,
                             const Angle &chip_direction, double chip_distance_meters)
    : robot_id(robot_id),
      chip_origin(chip_origin),
      chip_direction(chip_direction),
      chip_distance_meters(chip_distance_meters)
{
}

ChipPrimitive::ChipPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id             = primitive_msg.robot_id;
    double chip_origin_x = primitive_msg.parameters.at(0);
    double chip_origin_y = primitive_msg.parameters.at(1);
    chip_origin          = Point(chip_origin_x, chip_origin_y);
    chip_direction       = Angle::ofRadians(primitive_msg.parameters.at(2));
    chip_distance_meters = primitive_msg.parameters.at(3);
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

std::vector<double> ChipPrimitive::getParameters() const
{
    std::vector<double> parameters = {chip_origin.x(), chip_origin.y(),
                                      chip_direction.toRadians(), chip_distance_meters};

    return parameters;
}

std::vector<bool> ChipPrimitive::getExtraBits() const
{
    return std::vector<bool>();
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
