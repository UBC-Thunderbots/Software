#pragma once

#include "software/geom/generic_angle.h"

class AngularVelocity : public GenericAngle<AngularVelocity>
{
   public:
    constexpr AngularVelocity() = default;
    explicit constexpr AngularVelocity(double rad);
};

constexpr AngularVelocity::AngularVelocity(double rad) : GenericAngle(rad) {}
