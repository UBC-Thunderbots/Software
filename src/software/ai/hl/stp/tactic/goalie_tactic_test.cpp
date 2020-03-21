#include "software/ai/hl/stp/tactic/goalie_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/action/chip_action.h"

// The following tests will make sure the goalie stays in the requested
// deflated defense area when best positioning to defend the ball.
// The diagram below shows the requested position (R) by the goalie to where
// it should be defending, the restrain function returns the actual position (A)
// where the goalie is allowed to stay.
//
//
//    |
//    |defense
//    +-----------------+
//    |restrain         |
//    +-------+         |
// +--+       |         |
// |  |       |    R    |
// |  |      A|         |
// |  |       |         |
// |  |       |         |
// |  |       |         |
// |  |       |         |
// +--+       |         |
//    +-------+         |
//    |                 |
//    +-----------------+
//    |
//    |

class GoalieRestrainTest : public ::testing::TestWithParam<Point>
{
};

TEST_P(GoalieRestrainTest, goalie_position_safe)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    GoalieTactic tactic = GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                                       world.enemyTeam());

    // test to make sure that points given outside of the rectangle
    // are constrained inside
    auto small_rectangle = world.field().friendlyDefenseArea();
    small_rectangle.expand(-0.8);
    auto requested_position = GetParam();
    auto restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, small_rectangle);

    // scaling the restrained position by a slight bit as containsPoint does not count
    // the points right on the edge of the rectangle. For the purposes of the goalie
    // we are okay if the point is right on the edge, or close enough.
    EXPECT_TRUE(small_rectangle.contains((*restrained_position)));
    EXPECT_FALSE(small_rectangle.contains(requested_position));

    // test to make sure that points given inside of the rectangle
    // are not altered and are the same points
    // are constrained inside.
    auto big_rectangle = world.field().friendlyDefenseArea();

    // blow up rectangle to a huge amount, to contain all the points
    big_rectangle.expand(5);
    restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, big_rectangle);

    EXPECT_TRUE(big_rectangle.contains(requested_position));
    EXPECT_TRUE(big_rectangle.contains(*restrained_position));
}

INSTANTIATE_TEST_CASE_P(Positions, GoalieRestrainTest,
                        ::testing::Values(Point(0, 0), Point(1, 2), Point(0, 2),
                                          Point(1, 1), Point(0.5, 1), Point(1, 0.5)));

//coords friendly side is -x
//+y is up, -y is down
//fast is > 0.2
// deflation of goal is 0.2
// move equality is destination, orientation, dribbler_enable, ball_collsion, autochip, movetype, final speed
TEST(GoalieTacticTest, ball_very_fast_in_straight_line)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(4,0), Vector(-4,0),
            Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4, -1), Vector(0, 0),
            Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    tactic.updateRobot(goalie);
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    std::cout << move_action->getDestination() << std::endl;
    EXPECT_TRUE(move_action->getDestination().isClose(Point(0, 0), 0.05));
}

TEST(GoalieTacticTest, ball_very_fast_in_diagonal_line)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    ::Test::TestUtil::setBallPosition(world, Point(0,0), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setBallVelocity(world, Vector(4, -0.5), Timestamp::fromSeconds(0));
    ::Test::TestUtil::setEnemyRobotPositions(world, {Point(0.09, 0)}, Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4, -0.5), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().updateRobots({goalie});
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(Point(-4,0.5), 0.05));
}

TEST(GoalieTacticTest, ball_slow_inside_restrained_area)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(-4,0), Vector(-0.01,1), Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    tactic.updateRobot(goalie);
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto stop_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    ASSERT_NE(stop_action, nullptr);
}

TEST(GoalieTacticTest, ball_slow_outside_restrained_area)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(-3.5, 0.5), Vector(-0.01, -0.5), Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    tactic.updateRobot(goalie);
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
    //ASSERT_NE(chip_action, nullptr);
    // check chip equality
}

TEST(GoalieTacticTest, ball_miss)
{
    //expect move_action to close point

    // cases:
    // restrict to semicircle inside defense area
    // restrict to inside defense area
    // normal
}

TEST(GoalieTacticTest, ball_behind_net) //snap to closer goal post
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(-5.8,0.3), Vector(0.5, 0.5), Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    tactic.updateRobot(goalie);
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(Point(-4.59, 0.5), 0.01));
}

TEST(GoalieTacticTest, ball_angle_very_sharp_and_below_panic) // snap to closer goal post
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world.mutableBall() = Ball(Point(-4.5, -3), Vector(0, 0.1), Timestamp::fromSeconds(0));

    Robot goalie = Robot(0, Point(-4.5, 0), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableFriendlyTeam().assignGoalie(0);

    GoalieTactic tactic =
            GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                         world.enemyTeam());
    tactic.updateRobot(goalie);
    auto action_ptr = tactic.getNextAction();

    EXPECT_TRUE(action_ptr);

    auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
    ASSERT_NE(move_action, nullptr);
    EXPECT_TRUE(move_action->getDestination().isClose(Point(-4.59, -0.5), 0.01));
}