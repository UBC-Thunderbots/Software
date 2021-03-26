#include "penalty_goalie_tactic.h"

#include <gtest/gtest.h>

#include <utility>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"
#include "software/test_util/test_util.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"

class PenaltyGoalieRestrainTest : public ::testing::TestWithParam<Point>
{
};

TEST_P(PenaltyGoalieRestrainTest, goalie_position_safe)
{
    World world         = ::TestUtil::createBlankTestingWorld();
    PenaltyGoalieTactic tactic = PenaltyGoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                                       world.enemyTeam());

    auto requested_position = GetParam();

    const auto init_ball_position = world.ball().position();

    auto restrained_position =
            tactic.restrainPenaltyGoalieInGoalLine(requested_position);

    auto friendly_goal_upper_y = world.field().friendlyGoalCenter().y() + world.field().goalYLength() / 2;
    auto friendly_goal_lower_y = world.field().friendlyGoalCenter().y() - world.field().goalYLength() / 2;

    // scaling the restrained position by a slight bit as contains does not count
    // the points right on the edge of the rectangle. For the purposes of the goalie
    // we are okay if the point is right on the edge, or close enough.
    EXPECT_TRUE(restrained_position.y() <= friendly_goal_upper_y);
    EXPECT_TRUE(restrained_position.y() >= friendly_goal_lower_y);
    EXPECT_TRUE(restrained_position.x() == world.field().friendlyGoalCenter().x());


}

class PenaltyGoalieTacticTest : public testing::Test
{
protected:
    void expectMoveAction(Ball ball, Point destination)
    {
        World world = ::TestUtil::createBlankTestingWorld();
        world.updateBall(ball);

        Robot goalie = Robot(0, Point(-4.5, 0), Vector(0, 0), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));

        Team new_team({goalie});
        new_team.assignGoalie(0);
        world.updateFriendlyTeamState(new_team);

        PenaltyGoalieTactic tactic = PenaltyGoalieTactic(world.ball(), world.field(),
                                           world.friendlyTeam(), world.enemyTeam());
        tactic.updateRobot(goalie);
        auto action_ptr = tactic.getNextAction();

        EXPECT_TRUE(action_ptr);

        auto move_action = std::dynamic_pointer_cast<MoveAction>(action_ptr);
        ASSERT_NE(move_action, nullptr);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_action->getDestination(),
                                                   destination, 0.03));
        EXPECT_NEAR(move_action->getFinalSpeed(), 0, 0.001);
    }

    void expectStopAction(Ball ball)
    {
        World world = ::TestUtil::createBlankTestingWorld();
        world.updateBall(ball);

        Robot goalie = Robot(0, Point(-4.5, 0), Vector(0, 0), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Team new_team({goalie});
        new_team.assignGoalie(0);
        world.updateFriendlyTeamState(new_team);

        PenaltyGoalieTactic tactic = PenaltyGoalieTactic(world.ball(), world.field(),
                                           world.friendlyTeam(), world.enemyTeam());
        tactic.updateRobot(goalie);
        auto action_ptr = tactic.getNextAction();

        EXPECT_TRUE(action_ptr);

        auto stop_action = std::dynamic_pointer_cast<StopAction>(action_ptr);
        ASSERT_NE(stop_action, nullptr);
    }

    void expectChipAction(Ball ball)
    {
        World world = ::TestUtil::createBlankTestingWorld();
        world.updateBall(ball);

        Robot goalie = Robot(0, Point(-4.5, 0), Vector(0, 0), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
        Team new_team({goalie});
        new_team.assignGoalie(0);
        world.updateFriendlyTeamState(new_team);

        PenaltyGoalieTactic tactic = PenaltyGoalieTactic(world.ball(), world.field(),
                                           world.friendlyTeam(), world.enemyTeam());
        tactic.updateRobot(goalie);
        auto action_ptr = tactic.getNextAction();

        EXPECT_TRUE(action_ptr);

        auto chip_action = std::dynamic_pointer_cast<ChipAction>(action_ptr);
        ASSERT_NE(chip_action, nullptr);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(chip_action->getChipOrigin(),
                                                   world.ball().position(), 0.001));
        EXPECT_EQ(
                chip_action->getChipDirection(),
                (world.ball().position() - world.field().friendlyGoalCenter()).orientation());
        EXPECT_NEAR(chip_action->getChipDistanceMeters(), 2, 0.001);
    }
}

TEST_F(PenaltyGoalieTacticTest, ball_very_fast_in_straight_line)
{
    Ball ball = Ball(Point(0, 0), Vector(-4, 0), Timestamp::fromSeconds(0));
    expectMoveAction(ball, Point(-4.5, 0));
}

TEST_F(PenaltyGoalieTacticTest, ball_very_fast_in_diagonal_line)
{
    Ball ball = Ball(Point(0, 0), Vector(-4.5, 0.25), Timestamp::fromSeconds(0));
    expectMoveAction(ball, Point(-4.5, 0.25));
}

TEST_F(PenaltyGoalieTacticTest, ball_very_fast_miss)
{
    Ball ball = Ball(Point(0, 0), Vector(-4.5, 1), Timestamp::fromSeconds(0));
    // Goalie is expected to default to the centre of goal
    expectMoveAction(ball, Point(-3.7, 0));
}

TEST_F(PenaltyGoalieTacticTest, ball_slow_inside_dont_chip_rectangle)
{
    Ball ball = Ball(Point(-4.5 + ROBOT_MAX_RADIUS_METERS, 0), Vector(-0.1, 0.1),
                     Timestamp::fromSeconds(0));
    expectStopAction(ball);
}

TEST_F(PenaltyGoalieTacticTest, ball_slow_outside_dont_chip_rectangle)
{
    Ball ball = Ball(Point(-3.5, 0.5), Vector(-0.1, -0.1), Timestamp::fromSeconds(0));

    expectChipAction(ball);
}

TEST_F(PenaltyGoalieTacticTest, ball_behind_net_and_moving_toward_net)
{
    Ball ball = Ball(Point(-5.8, 0.3), Vector(0.5, 0.5), Timestamp::fromSeconds(0));
    // snap to closer goal post
    expectMoveAction(ball, Point(-4.5, 0.5 - ROBOT_MAX_RADIUS_METERS));
}

TEST_F(PenaltyGoalieTacticTest, ball_angle_very_sharp_and_low_velocity)
{
    Ball ball = Ball(Point(-4.5, -3), Vector(0, 0.1), Timestamp::fromSeconds(0));
    // snap to closer goal post
    expectMoveAction(ball, Point(-4.5, -0.5 + ROBOT_MAX_RADIUS_METERS));
}

TEST_F(PenaltyGoalieTacticTest, ball_far_away_and_zero_velocity)
{
    Ball ball = Ball(Point(4.5, 1), Vector(0, 0), Timestamp::fromSeconds(0));
    // (-4.5, 0) is friendly goal (-3.7, 0.8), (-3.7, -0.8) is defense area
    std::optional<Point> goalie_intersection = intersection(
            Line(Point(4.5, 1), Point(-4.5, 0)), Line(Point(-3.7, 0.8), Point(-3.7, -0.8)));
    expectMoveAction(ball, *goalie_intersection);
}

TEST_F(PenaltyGoalieTacticTest, ball_outside_defense_area_and_zero_velocity)
{
    Ball ball = Ball(Point(-3, -0.5), Vector(0, 0), Timestamp::fromSeconds(0));
    // (-4.5, 0) is friendly goal (-3.7, 0.8), (-3.7, -0.8) is defense area
    std::optional<Point> goalie_intersection = intersection(
            Line(Point(-3, -0.5), Point(-4.5, 0)), Line(Point(-3.7, 0.8), Point(-3.7, -0.8)));
    expectMoveAction(ball, *goalie_intersection);
}

TEST_F(PenaltyGoalieTacticTest, ball_in_defense_area_and_zero_velocity)
{
    Ball ball = Ball(Point(-4, -0.5), Vector(0, 0), Timestamp::fromSeconds(0));
    expectChipAction(ball);
}