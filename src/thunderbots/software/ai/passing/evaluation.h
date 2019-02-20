// TODO: start of file comment

// TODO: if this file (or the corresponding `.cpp`) gets too big, they may need to be broken up
#pragma once

#include "ai/world/field.h"
#include "geom/point.h"

/**
 * Calculates the static position quality for a given position on a given field
 *
 * @param field The field on which to calculate the static position quality
 * @param position The position on the field at which to calculate the quality
 *
 * @return A value in [0,1] representing the quality of the given point on the given field
 */
double getStaticPositionQuality(Field field, Point position);

