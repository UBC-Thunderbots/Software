

#include "software/geom/angle_segment.h"

AngleSegment::AngleSegment(Angle angle_top, Angle angle_bottom)
    : angle_top_(angle_top), angle_bottom_(angle_bottom)
{
}

const Angle &AngleSegment::getAngleTop() const
{
    return angle_top_;
}

void AngleSegment::setAngleTop(const Angle &angle_top)
{
    this->angle_top_ = angle_top;
}

const Angle &AngleSegment::getAngleBottom() const
{
    return angle_bottom_;
}

void AngleSegment::setAngleBottom(const Angle &angle_bottom)
{
    this->angle_bottom_ = angle_bottom;
}

double AngleSegment::getDelta() const
{
    return (angle_bottom_ - angle_top_).abs().toDegrees();
}

bool AngleSegment::operator==(const AngleSegment &other) const
{
    return getAngleTop() == other.getAngleTop();
}

bool AngleSegment::operator<(const AngleSegment &other) const
{
    return getAngleTop() < other.getAngleTop();
}

bool AngleSegment::operator>(const AngleSegment &other) const
{
    return getAngleTop() > other.getAngleTop();
}
