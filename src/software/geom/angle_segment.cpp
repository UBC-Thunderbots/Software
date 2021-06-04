

#include "software/geom/angle_segment.h"

/**
 * Constructs an AngleSegment represented by a top angle and bottom angle
 *
 * @param angle_top the most north angle
 * @param angle_bottom the most south angle
 */
AngleSegment::AngleSegment(Angle angle_top, Angle angle_bottom) : angle_top_(angle_top), angle_bottom_(angle_bottom)
{
}

/**
 * Gets the top (most north) angle of the angle segment
 *
 * @return the top angle of the angle segment
 */
const Angle &AngleSegment::getAngleTop() const
{
    return angle_top_;
}

/**
 * Sets the top (most north) angle of the angle segment
 *
 * @param angle_top the top angle of the angle segment
 */
void AngleSegment::setAngleTop(const Angle &angle_top)
{
    this->angle_top_ = angle_top;
}

/**
 * Gets the bottom (most south) angle of the angle segment
 *
 * @return the bottom angle of the angle segment
 */
const Angle &AngleSegment::getAngleBottom() const
{
    return angle_bottom_;
}

/**
 * Sets the bottom (most south) angle of the angle segment
 *
 * @param angle_bottom the bottom angle of the angle segment
 */
void AngleSegment::setAngleBottom(const Angle &angle_bottom)
{
    this->angle_bottom_ = angle_bottom;
}

/**
 * Gets the abs delta between the two angles that describe this angle segment
 *
 * @return the abs delta between the top and bottom angles
 */
double AngleSegment::getDelta() const
{
    return (angle_bottom_ - angle_top_).abs().toDegrees();
}
