#include "software/geom/algorithms/end_in_obstacle_sample.h"

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/point.h"


std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr>& obstacles,
                                         const Point &point,
                                         const Rectangle &navigable_area,
                                         double radius_step, int samples_per_radius_step,
                                         double max_search_radius)
{
    // first, check if point is inside an obstacle
    bool point_in_obstacle = false;
    for (auto const &obstacle : obstacles)
    {
        if (obstacle->contains(point) || !contains(navigable_area, point))
        {
            if (!point_in_obstacle)
            {
                point_in_obstacle = true;
            }

            // if point is inside obstacle, perform a second check to see if the closest point outside the first encroached obstacle is inside another obstacle
            Point closest_point            = obstacle->closestPoint(point);
            bool closest_point_in_obstacle = false;
            // break out of loop if closest point outside first encroached obstacle is outside navigable area
            if (!contains(navigable_area, closest_point))
            {
                closest_point_in_obstacle = true;
                break;
            }
            for (auto const &obstacle : obstacles)
            {
                // break out of loop if closest point outside first encroached obstacle happens to be inside another obstacle
                if (obstacle->contains(closest_point))
                {
                    closest_point_in_obstacle = true;
                    break;
                }
            }

            if (closest_point_in_obstacle)
            {
                break;
            }
            else
            {
                // if the closest point outside the first encroached obstacle is not inside any other obstacle, then return it
                return closest_point;
            }
        }
    }

    // if provided point isn't inside an obstacle, just return point as is
    if (!point_in_obstacle)
    {
        return point;
    }

    // perform sampling only if the provided point or the closest point outside the first encroached obstacle are not
    // valid
    double radius        = 0.15;
    int samples_per_radius = 6;
    while (radius <= max_search_radius)
    {
        double increment = 360.0 / samples_per_radius;
        for (int i = 0; i < samples_per_radius; i++)
        {
            Angle angle        = Angle::fromDegrees(static_cast<float>(i) * increment);
            Vector direction   = Vector::createFromAngle(angle);
            Point sample_point = point + direction * radius;
            bool sample_point_in_obstacle = false;
            for (auto const &obstacle : obstacles)
            {
                if (obstacle->contains(sample_point) ||
                    !contains(navigable_area, sample_point))
                {
                    sample_point_in_obstacle = true;
                    break;
                }
            }
            if (!sample_point_in_obstacle)
            {
                return sample_point;
            }
        }
        // increase the number of samples per radius to prevent density of samples from dropping off
        samples_per_radius += samples_per_radius_step;
        radius += radius_step;
    }
    return std::nullopt;
}
