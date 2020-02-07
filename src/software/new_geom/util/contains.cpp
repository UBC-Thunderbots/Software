#include "software/new_geom/util/contains.h"

#include "software/new_geom/util/collinear.h"

bool contains(const Segment &segment, const Point &point)
{
    if (collinear(point, segment.getSegStart(), segment.getEnd()))
    {
        // If the segment and point are in a perfect vertical line, we must use Y
        // coordinate centric logic
        if ((std::abs(point.x() - segment.getEnd().x()) < GeomConstants::EPSILON) &&
            (std::abs(segment.getEnd().x() - segment.getSegStart().x()) <
             GeomConstants::EPSILON))
        {
            // Since segment and point are collinear we only need to check one of the
            // coordinates, in this case we select Y because all X values are equal
            return (point.y() <= segment.getSegStart().y() &&
                    point.y() >= segment.getEnd().y()) ||
                   (point.y() <= segment.getEnd().y() &&
                    point.y() >= segment.getSegStart().y());
        }

        // Since segment and point are collinear we only need to check one of the
        // coordinates, choose x because we know there is variance in these values
        return (point.x() <= segment.getSegStart().x() &&
                point.x() >= segment.getEnd().x()) ||
               (point.x() <= segment.getEnd().x() &&
                point.x() >= segment.getSegStart().x());
    }

    return false;
}

bool contains(const Ray &ray, const Point &point)
{
    Point point_in_ray_direction = ray.getStart() + ray.toUnitVector();
    return (point == ray.getStart()) ||
           (collinear(point, ray.getStart(), point_in_ray_direction) &&
            (((point - ray.getStart()).normalize() - ray.toUnitVector()).length() <
             GeomConstants::EPSILON));
}
