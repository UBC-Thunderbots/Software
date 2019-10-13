#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/intent/stop_intent.h"
#include "software/test_util/test_util.h"

TEST(StopTacticTest, robot_stopping_without_coasting_while_already_moving)
{
    Robot robot       = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopTactic tactic = StopTactic(false, false);
    tactic.updateRobot(robot);

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());

    try
    {
        StopIntent stop_intent = dynamic_cast<StopIntent &>(*intent_ptr);
        EXPECT_EQ(0, stop_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "StopIntent was not returned by the StopTactic!";
    }
}

TEST(StopTacticTest, robot_stopping_while_already_stopped)
{
    Robot robot       = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopTactic tactic = StopTactic(false, false);
    tactic.updateRobot(robot);

    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(intent_ptr);
    EXPECT_FALSE(tactic.done());

    try
    {
        StopIntent stop_intent = dynamic_cast<StopIntent &>(*intent_ptr);
        EXPECT_EQ(0, stop_intent.getRobotId());
    }
    catch (...)
    {
        ADD_FAILURE() << "StopIntent was not returned by the StopTactic!";
    }
}

TEST(StopTacticTest, test_calculate_robot_cost)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    StopTactic tactic = StopTactic(false, false);

    // We always expect the cost to be 0.5, because the StopTactic prefers all robots
    // equally
    EXPECT_EQ(0.5, tactic.calculateRobotCost(robot, world));
}
