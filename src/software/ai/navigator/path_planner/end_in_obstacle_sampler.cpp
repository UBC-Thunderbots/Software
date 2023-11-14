#include <random>
#include "software/ai/navigator/path_planner/end_in_obstacle_sampler.h"

EndInObstacleSampler::EndInObstacleSampler(const std::vector<ObstaclePtr> obstacles, const Point &destination)
: obstacles(obstacles), destination(destination)
{}

void EndInObstacleSampler::compute()
{   bool destination_in_obstacle = false;
    for (auto const &obstacle : obstacles) {
        if (obstacle->contains(destination)) {
            destination_in_obstacle = true;
            Point closest_point = closestPoint(destination, obstacle.getg);
            break;
        }
    }

    if (!destination_in_obstacle) {
        // return point
    }


}

void EndInObstacleSampler::randomSample()
{
    const int ITERATIONS = 30;
    std::mt19937 rng;
    std::uniform_real_distribution<> rad_uniform_dist(0, 30);
    std::uniform_real_distribution<> ang_uniform_dist(0, 360);

    int rad_min = 0;
    for (int i = 0; i < ITERATIONS; i++) {
        Angle angle = Angle::fromDegrees(ang_uniform_dist(rng));
        Vector dir = Vector::createFromAngle(angle);
        double rad = rad_min + rad_uniform_dist(rng);

        Point sample_point = destination + dir * rad;
        for (auto const &obstacle : obstacles) {
            if (!obstacle->contains(sample_point)) {

            }
        }
    }
}

void EndInObstacleSampler::circularSample()
{
    const int STEP = 15;
    int steps = 360 / STEP;
    int rad = 10;


    for (int i = 0; i < steps; i++) {
        Angle angle = Angle::zero();

    }
}