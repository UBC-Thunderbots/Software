#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"
#include "software/test_util/test_util.h"

TEST(MoveTacticTest, robot_far_from_destination)
{
    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic();
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(1, 0), Angle::quarter(), 1.0);
    auto intent_ptr = tactic.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);

    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
    EXPECT_EQ(0, move_intent.getRobotId());
    EXPECT_EQ(Point(1, 0), move_intent.getDestination());
    EXPECT_EQ(Angle::quarter(), move_intent.getFinalAngle());
    EXPECT_EQ(1.0, move_intent.getFinalSpeed());
}

TEST(MoveTacticTest, robot_at_destination)
{
    Robot robot = Robot(0, Point(), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic();
    tactic.updateRobot(robot);
    tactic.updateControlParams(Point(0, 0), Angle::zero(), 0.0);

    // We call the Tactic twice. The first time the Intent will always be returned to
    // ensure the Robot is doing the right thing. In all future calls, the action will be
    // done and so will return a null pointer
    auto intent_ptr = tactic.getNextIntent();
    intent_ptr      = tactic.getNextIntent();

    EXPECT_TRUE(tactic.done());
}

TEST(MoveTacticTest, test_calculate_robot_cost)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());

    Robot robot = Robot(0, Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    MoveTactic tactic = MoveTactic();
    tactic.updateControlParams(Point(3, -4), Angle::zero(), 0.0);

    EXPECT_EQ(5 / world.field().totalXLength(), tactic.calculateRobotCost(robot, world));
}
