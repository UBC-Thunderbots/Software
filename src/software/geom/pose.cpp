#include "software/geom/pose.h"

Pose::Pose() : position_(Point()), orientation_(Angle::zero()) {}

Pose::Pose(const Point &position, const Angle orientation)
    : position_(position), orientation_(orientation)
{
}

const Point &Pose::position() const
{
    return position_;
}

const Angle &Pose::orientation() const
{
    return orientation_;
}

bool Pose::operator==(const Pose &rhs) const
{
    return position_ == rhs.position_ && orientation_ == rhs.orientation_;
}

bool Pose::operator!=(const Pose &rhs) const
{
    return !(rhs == *this);
}

std::ostream &operator<<(std::ostream &os, const Pose &p)
{
    os << p.position() << " @ " << p.orientation();
    return os;
}
