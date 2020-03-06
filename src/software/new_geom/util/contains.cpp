#include "software/new_geom/util/contains.h"

#include "software/new_geom/util/distance.h"

bool containsNew(const Circle &out, const Segment &in)
{
    return distance(in, out.getOrigin()) < out.getRadius();
}
