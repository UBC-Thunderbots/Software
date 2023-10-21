#include "software/geom/angle.h"

/**
 * We also use variables of type 'Angle' to represent angular velocities, since they
 * are essentially represented the same. This typedef allows us to refer to Angles as
 * AngularVelocities, which makes the interfaces more intuitive.
 * @note Not all methods of Angle class make sense for AngularVelocity!
 * E.g. Angle::clamp does not make sense in the context of AngularVelocity, as 360 deg/s
 * is different from 0 deg/s, but 360 deg is the same as 0 deg.
 */
using AngularVelocity = Angle;
