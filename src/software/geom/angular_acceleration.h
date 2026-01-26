#pragma once

#include "software/geom/generic_angle.h"

class AngularAcceleration : public GenericAngle<AngularAcceleration>
{
   public:
    constexpr AngularAcceleration() = default;
    explicit constexpr AngularAcceleration(double rad);
};

constexpr AngularAcceleration::AngularAcceleration(double rad) : GenericAngle(rad) {}
