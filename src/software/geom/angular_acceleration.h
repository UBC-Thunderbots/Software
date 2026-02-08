#include "software/geom/angle.h"

/**
 * A typesafe representation of an angular acceleration.
 *
 * This class is an instantiation of the GenericAngle template class. Used to
 * differentiate between the Angle class while keeping most of the common
 * functionality that makes sense.
 */
using AngularAcceleration = GenericAngle<AngularAccelerationTag>;
