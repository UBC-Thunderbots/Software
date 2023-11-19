#include <random>
#include "software/ai/navigator/path_planner/end_in_obstacle_sampler.h"

EndInObstacleSampler::EndInObstacleSampler(float rad_multiplier, float step_multiplier)
: rad_multiplier(rad_multiplier), step_multiplier(step_multiplier)
{}

void EndInObstacleSampler::setRadMultiplier(float multiplier) {
    rad_multiplier = multiplier;
}

void EndInObstacleSampler::setStepMultiplier(float multiplier) {
    step_multiplier = multiplier;
}

Point EndInObstacleSampler::compute(const std::vector<ObstaclePtr> obstacles, const Point &destination)
{   bool destination_in_obstacle = false;
    for (auto const &obstacle : obstacles) {
        if (obstacle->contains(destination)) {
            destination_in_obstacle = true;

            Point closest_point = obstacle->closestPoint(destination); //
            for (auto const &obstacle : obstacles) {
                if (obstacle->contains(closest_point)) {
                    goto sampling_required;
                }
            }
            return closest_point;
        }
    }
    return destination;

sampling_required:
    return circularSample(obstacles, destination, step_multiplier);
}

//void EndInObstacleSampler::randomSample()
//{
//    const int ITERATIONS = 30;
//    std::mt19937 rng;
//    std::uniform_real_distribution<> rad_uniform_dist(0, 150);
//    std::uniform_real_distribution<> ang_uniform_dist(0, 360);
//
//    int rad_min = 0;
//    for (int i = 0; i < ITERATIONS; i++) {
//        Angle angle = Angle::fromDegrees(ang_uniform_dist(rng));
//        Vector dir = Vector::createFromAngle(angle);
//        double rad = rad_min + rad_uniform_dist(rng);
//
//        Point sample_point = destination + dir * rad;
//        for (auto const &obstacle : obstacles) {
//            if (!obstacle->contains(sample_point)) {
//
//            }
//        }
//    }
//}

Point EndInObstacleSampler::circularSample(const std::vector<ObstaclePtr> obstacles, const Point& destination, float range)
{
    float rad = 0.15;
    int steps = 6;
    float shortest_distance = range;
    Point end_point;

    while (rad <= range) {
        float increment = 360 / steps;
        for (int i = 0; i < steps; i++) {
            Angle angle = Angle::fromDegrees(i * increment);
            Vector direction = Vector::createFromAngle(angle);
            Point point = destination + direction * rad;
            if (distance(point, destination) < shortest_distance) {
                shortest_distance = distance(point, destination);
                end_point = point;
            }
        }
        steps *= step_multiplier;
        rad += 0.20;
    }

    return end_point;
}