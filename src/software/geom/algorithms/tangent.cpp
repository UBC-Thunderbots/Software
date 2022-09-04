#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/almost_equal.h"
#include "software/geom/algorithms/collinear.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/circle.h"
#include "software/geom/line.h"


std::vector<Line> getTangentLines(const Circle &circle, const Point &point)
{
    std::vector<Line> res;

    if (contains(circle, point))
    {
        if (distance(point, circle.origin()) == circle.radius())
        {
            Vector perp =
                Line(circle.origin(), point).toNormalUnitVector().perpendicular();
            Line tangent = Line(point, point + perp);
            res.push_back(tangent);
        }
        else
        {
            // point inside a circle has no tangent lines to the circle
            return res;
        }
    }
    else
    {
        Vector to_origin = circle.origin() - point;
        Vector perp1     = to_origin.perpendicular();
        Vector perp2     = perp1 * -1;

        Point tangent_point1 = circle.origin() + perp1.normalize(circle.radius());
        Point tangent_point2 = circle.origin() + perp2.normalize(circle.radius());

        res.push_back(Line(point, tangent_point1));
        res.push_back(Line(point, tangent_point2));
    }
    return res;
}
