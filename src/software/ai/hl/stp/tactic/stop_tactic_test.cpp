#include "software/ai/hl/stp/tactic/stop_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/intent/stop_intent.h"
#include "software/test_util/test_util.h"

TEST(StopTacticTest, robot_stopping_without_coasting_while_already_moving)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot       = Robot(0, Point(0, 0), Vector(2, -1), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopTactic tactic = StopTactic(false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());

    auto stop_action = std::dynamic_pointer_cast<StopAction>(action_ptr);
    ASSERT_NE(nullptr, stop_action);
    ASSERT_TRUE(stop_action->getRobot().has_value());
    EXPECT_EQ(0, stop_action->getRobot()->id());
}

TEST(StopTacticTest, robot_stopping_while_already_stopped)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot       = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    StopTactic tactic = StopTactic(false);
    tactic.updateRobot(robot);
    tactic.updateWorldParams(world);

    auto action_ptr = tactic.getNextAction();

    // Check an intent was returned (the pointer is not null)
    ASSERT_TRUE(action_ptr);
    EXPECT_FALSE(tactic.done());

    auto stop_action = std::dynamic_pointer_cast<StopAction>(action_ptr);
    ASSERT_NE(nullptr, stop_action);
    ASSERT_TRUE(stop_action->getRobot().has_value());
    EXPECT_EQ(0, stop_action->getRobot()->id());
}

TEST(StopTacticTest, test_calculate_robot_cost)
{
    World world = ::TestUtil::createBlankTestingWorld();

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    StopTactic tactic = StopTactic(false);
    tactic.updateWorldParams(world);

    // We always expect the cost to be 0.5, because the StopTactic prefers all robots
    // equally
    EXPECT_EQ(0.5, tactic.calculateRobotCost(robot, world));
}
