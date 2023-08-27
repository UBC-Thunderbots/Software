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
    traj.generate(Point(0, 0), Point(1, 3), Vector(3.0, 0.0), 3, 3, 6);

    const int num_points = 20;
    for (int i = 0; i <= num_points; ++i)
    {
        Point pos = traj.getPosition(Duration::fromSeconds(i * traj.getTotalTime().toSeconds() / num_points));
        std::cout << pos << ",";
    }
    std::cout << std::endl;
}
