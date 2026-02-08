#include "software/geom/angle.h"

/**
 * A typesafe representation of an angular velocity.
 *
 * This class is an instantiation of the GenericAngle template class. Used to
 * differentiate between the Angle class while keeping most of the common
 * functionality that makes sense.
 */
using AngularVelocity = GenericAngle<AngularVelocityTag>;
