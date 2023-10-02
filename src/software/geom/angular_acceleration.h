#include "software/geom/angle.h"
/**
 * We also use variables of type 'Angle' to represent angular acceleration, since they
 * are essentially represented the same. This typedef allows us to refer to Angles as
 * AngularAcceleration, which makes the interfaces more intuitive.
 * @note Not all methods of Angle class make sense for AngularAcceleration!
 * E.g. Angle::clamp does not make sense in the context of AngularAcceleration, as
 * 360 deg/s^2 is different from 0 deg/s^2, but 360 deg is the same as 0 deg.
 */
typedef Angle AngularAcceleration;
