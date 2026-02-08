#pragma once

#include "software/geom/generic_angle.h"

/**
 * A typesafe representation of an angle.
 *
 * This class is an instantiation of the GenericAngle template class. Used to
 * define more methods such as clamp, minDiff, etc. that only apply to a base
 * angle value.
 */
using Angle = GenericAngle<AngleTag>;
