#include <random>
#include "software/geom/algorithms/end_in_obstacle_sample.h"
#include "software/geom/point.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"


Point endInObstacleSample(const std::vector<ObstaclePtr> obstacles, const Point &destination, const Rectangle &navigable_area, double multiplier, double range)
{
    for (auto const &obstacle : obstacles) {
        if (obstacle->contains(destination)) {
            goto get_nearest;
        }
    }
    return destination;

get_nearest:
    for (auto const &obstacle : obstacles) {
        if (obstacle->contains(destination)) {

            Point closest_point = obstacle->closestPoint(destination);
            for (auto const &obstacle : obstacles) {
                if (obstacle->contains(closest_point)) {
                    goto do_sampling;
                }
            }
            return closest_point;
        }
    }

do_sampling:
    double rad = 0.15;
    int steps_per_rad = 6;
    while (rad <= range) {
        double increment = 360.0 / steps_per_rad;
        for (int i = 0; i < steps_per_rad; i++) {
            Angle angle = Angle::fromDegrees(float(i) * increment);
            Vector direction = Vector::createFromAngle(angle);
            Point sample_point = destination + direction * rad;
            for (auto const &obstacle : obstacles) {
                if (obstacle->contains(sample_point)) {
                    goto point_invalid;
                }
            }
            return sample_point;
point_invalid:;
        }
        steps_per_rad = int(steps_per_rad * multiplier);
        rad *= multiplier;
    }
    return destination;
}
