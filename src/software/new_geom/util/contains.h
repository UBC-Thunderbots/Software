#pragma once

#include "software/new_geom/circle.h"
#include "software/new_geom/segment.h"

/**
 * Returns whether the Circle contains the Segment
 * Segment is considered contained in Circle if both endpoints are inside the Circle
 *
 * @param out
 * @param in
 * @return true if the Circle contains the Segment, false otherwise
 */
bool containsNew(const Circle &out, const Segment &in);
