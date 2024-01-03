#include "software/geom/pose.h"

Pose::Pose() : point_(Point()), orientation_(Angle::zero()) {}

Pose::Pose(const Point& point, const Angle& orientation)
    : point_(point), orientation_(orientation)
{
}

Pose::Pose(double x, double y, const Angle& orientation)
    : point_(x, y), orientation_(orientation)
{
}

Point Pose::point() const
{
    return point_;
}

Angle Pose::orientation() const
{
    return orientation_;
}
