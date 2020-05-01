#include "software/primitive/pivot_primitive.h"

#include "shared/constants.h"
#include "software/primitive/primitive_visitor.h"

const std::string PivotPrimitive::PRIMITIVE_NAME = "Pivot Primitive";

PivotPrimitive::PivotPrimitive(unsigned int robot_id, const Point &pivot_point,
                               const Angle &final_angle, const Angle &pivot_speed,
                               bool enable_dribbler)
    : robot_id(robot_id),
      pivot_point(pivot_point),
      final_angle(final_angle),
      pivot_speed(pivot_speed),
      enable_dribbler(enable_dribbler)
{
}

std::string PivotPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int PivotPrimitive::getRobotId() const
{
    return robot_id;
}

Point PivotPrimitive::getPivotPoint() const
{
    return pivot_point;
}

Angle PivotPrimitive::getFinalAngle() const
{
    return final_angle;
}

Angle PivotPrimitive::getPivotSpeed() const
{
    return pivot_speed;
}

double PivotPrimitive::getPivotRadius() const
{
    return BALL_MAX_RADIUS_METERS;
}

bool PivotPrimitive::isDribblerEnabled() const
{
    return enable_dribbler;
}

void PivotPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool PivotPrimitive::operator==(const PivotPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->pivot_point == other.pivot_point &&
           this->final_angle == other.final_angle &&
           this->pivot_speed == other.pivot_speed &&
           this->enable_dribbler == other.enable_dribbler;
}

bool PivotPrimitive::operator!=(const PivotPrimitive &other) const
{
    return !((*this) == other);
}
