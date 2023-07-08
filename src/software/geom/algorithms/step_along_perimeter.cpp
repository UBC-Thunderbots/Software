#include "software/geom/algorithms/step_along_perimeter.h"

#include "software/geom/algorithms/contains.h"

std::optional<Point> stepAlongPerimeter(const Rectangle &rectangle, const Point &start,
                                        double distance)
{
    // This function hasn't been properly implemented yet, but it
    // works well enough for usage in CreaseDefenderFSM.
    //
    // When we reach a corner, we're supposed to continue stepping
    // along the next edge in the rectangle, but I can't get this
    // to work without the robots crashing. For now I've made it so
    // that we only step along the edge that the starting point was
    // originally on.
    //
    // Need to investigate after Robocup

    double remaining_distance = std::abs(distance);
    bool step_clockwise       = distance >= 0;
    Point step_point          = start;

    auto edge = edgeContainingPoint(rectangle, step_point);
    if (!edge)
    {
        return std::nullopt;
    }

    Vector step_vector;
    if (step_clockwise)
    {
        step_vector = edge->getEnd() - step_point;
    }
    else
    {
        step_vector = edge->getStart() - step_point;
    }

    if (step_vector.length() > remaining_distance)
    {
        step_vector = step_vector.normalize() * remaining_distance;
    }

    step_point += step_vector;
    remaining_distance -= step_vector.length();

    return std::make_optional(step_point);
}

std::optional<Segment> edgeContainingPoint(const Rectangle &rectangle, const Point &point)
{
    Segment leftEdge(rectangle.negXNegYCorner(), rectangle.negXPosYCorner());
    Segment rightEdge(rectangle.posXPosYCorner(), rectangle.posXNegYCorner());
    Segment topEdge(rectangle.negXPosYCorner(), rectangle.posXPosYCorner());
    Segment bottomEdge(rectangle.posXNegYCorner(), rectangle.negXNegYCorner());

    if (contains(leftEdge, point))
    {
        return std::make_optional(leftEdge);
    }
    else if (contains(rightEdge, point))
    {
        return std::make_optional(rightEdge);
    }
    else if (contains(topEdge, point))
    {
        return std::make_optional(topEdge);
    }
    else if (contains(bottomEdge, point))
    {
        return std::make_optional(bottomEdge);
    }

    return std::nullopt;
}
