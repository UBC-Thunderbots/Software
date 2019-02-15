
#include <gtest/gtest.h>

#include "ai/hl/stp/evaluation/team.h"

//using namespace Evaluation;

class EvaluationTeamTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        current_time = Timestamp::fromSeconds(123);
    }

    Timestamp current_time;
};

TEST_F(EvaluationTeamTest, one_robot)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);


    team.updateRobots({robot_0});

    EXPECT_EQ(robot_0, Evaluation::nearest_friendly(team, Point(0,0)));
}

TEST_F(EvaluationTeamTest, multiple_robots)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_1, Evaluation::nearest_friendly(team, Point(0,0)));
}

TEST_F(EvaluationTeamTest, multiple_robots_closest_is_moving)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_1, Evaluation::nearest_friendly(team, Point(0,0)));
}

TEST_F(EvaluationTeamTest, multiple_robots_all_moving)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(3, -1), Vector(3, 3), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(4, 6), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(0, 1), Vector(3, 3), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_2, Evaluation::nearest_friendly(team, Point(0,0)));
}

TEST_F(EvaluationTeamTest, one_robot_on_ball)
{
    Team team = Team(Duration::fromMilliseconds(1000));

    Robot robot_0 = Robot(0, Point(0, 0), Vector(), Angle::half(),
                          AngularVelocity::threeQuarter(), current_time);

    Robot robot_1 = Robot(1, Point(1, 0), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);

    Robot robot_2 = Robot(2, Point(4, 6), Vector(), Angle::zero(),
                          AngularVelocity::zero(), current_time);


    team.updateRobots({robot_0, robot_1, robot_2});

    EXPECT_EQ(robot_0, Evaluation::nearest_friendly(team, Point(0,0)));
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
