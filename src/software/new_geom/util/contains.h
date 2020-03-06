#pragma once

#include "software/new_geom/circle.h"
#include "software/new_geom/segment.h"

/**
 * Returns true if the Circle contains the Segment, false otherwise
 *
 * @param out
 * @param in
 * @return true if the Circle contains the Segment, false otherwise
 */
bool containsNew(const Circle &out, const Segment &in);
