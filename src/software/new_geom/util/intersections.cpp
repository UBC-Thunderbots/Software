#include "software/new_geom/util/intersections.h"

std::vector<Point> intersection(const Segment &first, const Segment &second)
{
    std::vector<Point> output;
    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(), second.getEnd());
    boost::geometry::intersection(AB, CD, output);

    return output;
}
