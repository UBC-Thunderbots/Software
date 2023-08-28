#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class BangBangTrajectory2DTest : public testing::Test
{
   protected:
    BangBangTrajectory2D traj;
};

TEST_F(BangBangTrajectory2DTest, test1)
{
    traj.generate(Point(0, 0), Point(0, 0), Vector(0.0, 0.0), 1, 1, 1);

    std::cout << "Positions " << std::endl;
    const int num_points = 20;
    for (int i = 0; i <= num_points; ++i)
    {
        Point pos = traj.getPosition(
            Duration::fromSeconds(i * traj.getTotalTime().toSeconds() / num_points));
        std::cout << pos << ",";
    }
    std::cout << std::endl;


    std::cout << "XY vel" << std::endl;
    for (int i = 0; i <= num_points; ++i)
    {
        double time = i * traj.getTotalTime().toSeconds() / num_points;
        Vector vel  = traj.getVelocity(Duration::fromSeconds(time));
        std::cout << Point(time, vel.length()) << ",";
    }
    std::cout << std::endl;

    std::cout << "XY accel" << std::endl;
    for (int i = 0; i <= num_points; ++i)
    {
        double time = i * traj.getTotalTime().toSeconds() / num_points;
        Vector acc  = traj.getAcceleration(Duration::fromSeconds(time));
        std::cout << Point(time, acc.length()) << ",";
    }
    std::cout << std::endl;

    std::cout << "Total time: " << traj.getTotalTime().toSeconds() << std::endl;
}
