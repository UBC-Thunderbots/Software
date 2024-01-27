#include <random>
#include "software/geom/algorithms/end_in_obstacle_sample.h"
#include "software/geom/point.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"


std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr> obstacles, const Point &point, const Rectangle &navigable_area, int initial_count, double rad_step, int per_rad_step, double range)
{
    bool point_in_obstacle = false;
    for (auto const &obstacle : obstacles) {
        if (obstacle->contains(point) || !contains(navigable_area, point)) {
            if (!point_in_obstacle) {
                point_in_obstacle = true;
            }

            Point closest_point = obstacle->closestPoint(point);
            bool closest_point_in_obstacle = false;
            for (auto const &obstacle : obstacles) {
                if (obstacle->contains(closest_point)) {
                    closest_point_in_obstacle = true;
                    break;
                }
                if (!contains(navigable_area, closest_point)) {
                    closest_point_in_obstacle = true;
                    break;
                }
            }

            if (closest_point_in_obstacle) {
                break;
            } else {
                return closest_point;
            }
        }
    }

    if (!point_in_obstacle) {
        return point;
    }

    // sample if original point or closest point outside initial obstacle checked aren't valid
    double rad = 0.15;
    int steps_per_rad = 6;
    while (rad <= range) {
        double increment = 360.0 / steps_per_rad;
        for (int i = 0; i < steps_per_rad; i++) {
            Angle angle = Angle::fromDegrees(static_cast<float>(i) * increment);
            Vector direction = Vector::createFromAngle(angle);
            Point sample_point = point + direction * rad;
            bool sample_point_in_obstacle = false;
            for (auto const &obstacle : obstacles) {
                if (obstacle->contains(sample_point) || !contains(navigable_area, sample_point)) {
                    sample_point_in_obstacle = true;
                    break;
                }
            }
            if (!sample_point_in_obstacle) {
                return sample_point;
            }
        }
        steps_per_rad += per_rad_step;
        rad += rad_step;
    }
    return std::nullopt;
}
