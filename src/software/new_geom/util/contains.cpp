#include "software/new_geom/util/contains.h"

bool containsNew(const Circle &out, const Segment &in)
{
    return out.contains(in.getStart()) && out.contains(in.getEnd());
}
