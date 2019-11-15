#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/angular_velocity.h"
#include "software/util/time/timestamp.h"
#include "software/world/robot_capabilities.h"

class RobotState final
{
public:

    explicit RobotState(const Point &position, const Vector &velocity,
                        const Angle &orientation,
                        const AngularVelocity &angular_velocity,
                        const Timestamp &timestamp);

    Point position() const;

    Vector velocity() const;

    Angle orientation() const;

    AngularVelocity angularVelocity() const;

    Timestamp timestamp() const;

    bool operator==(const RobotState &other) const;

    bool operator!=(const RobotState &other) const;

private:

    Point position_;

    Vector velocity_;

    Angle orientation_;

    AngularVelocity angular_velocity_;

    Timestamp timestamp_;
};
