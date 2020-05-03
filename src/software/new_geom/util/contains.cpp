#include "software/new_geom/util/contains.h"

bool containsNew(const Circle &out, const Segment &in)
{
    return out.contains(in.getSegStart()) && out.contains(in.getEnd());
}
