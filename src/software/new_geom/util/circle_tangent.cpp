#include "software/new_geom/util/circle_tangent.h"

#include "software/new_geom/util/contains.h"

std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle)
{
    // If the point is already inside the circe arccos won't work so just return
    // the perp points
    if (contains(circle, start))
    {
        double perpDist = std::sqrt(circle.getRadius() * circle.getRadius() -
                                    (circle.getOrigin() - start).lengthSquared());
        Point p1 =
            start + (circle.getOrigin() - start).perpendicular().normalize(perpDist);
        Point p2 =
            start - ((circle.getOrigin() - start).perpendicular().normalize(perpDist));
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.getRadius() / (start - circle.getOrigin()).length());
        Point p1 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius());
        Point p2 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(-Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius());
        return std::make_pair(p1, p2);
    }
}

std::pair<Ray, Ray> getCircleTangentRaysWithReferenceOrigin(const Point reference,
                                                            const Circle circle)
{
    auto [tangent_point1, tangent_point2] = getCircleTangentPoints(reference, circle);

    return std::make_pair(Ray(reference, (tangent_point1 - reference)),
                          Ray(reference, (tangent_point2 - reference)));
}
