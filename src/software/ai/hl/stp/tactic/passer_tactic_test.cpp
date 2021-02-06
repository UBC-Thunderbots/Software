#include "software/ai/hl/stp/tactic/passer_tactic.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/hl/stp/action/intercept_ball_action.h"
#include "software/ai/hl/stp/action/kick_action.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/test_util/test_util.h"

TEST(
    PasserTacticTest,
    passer_already_at_pass_start_position_but_oriented_incorrectly_pass_not_yet_started_with_stationary_ball)
{
    // Robot is sitting at origin facing towards enemy goal
    Robot robot = Robot(13, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Line up behind the ball
    auto move_action = std::dynamic_pointer_cast<MoveAction>(tactic.getNextAction());
    ASSERT_TRUE(move_action);
    EXPECT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(13, move_action->getRobot()->id());
}

TEST(
    PasserTacticTest,
    passer_oriented_correctly_but_not_at_pass_start_position_pass_not_yet_started_with_stationary_ball)
{
    // Robot is sitting at {1,2} facing towards -y
    Robot robot = Robot(13, Point(1, 2), Vector(), Angle::fromDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Line up behind the ball
    auto move_action = std::dynamic_pointer_cast<MoveAction>(tactic.getNextAction());
    ASSERT_TRUE(move_action);
    EXPECT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(13, move_action->getRobot()->id());
}

TEST(
    PasserTacticTest,
    passer_oriented_incorrectly_and_close_to_start_position_but_in_front_of_pass_pass_not_yet_started_with_stationary_ball)
{
    // Robot is sitting at {-0.3,0.2} facing towards -y
    Robot robot = Robot(13, Point(-0.3, 0.2), Vector(), Angle::fromDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction. This means the
    // robot needs to move around the ball to get toa point where it can kick
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Line up behind the ball
    auto move_action = std::dynamic_pointer_cast<MoveAction>(tactic.getNextAction());
    ASSERT_TRUE(move_action);
    EXPECT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(13, move_action->getRobot()->id());
}

TEST(PasserTacticTest,
     passer_in_position_to_kick_pass_not_yet_started_with_stationary_ball)
{
    // Robot is sitting just behind where we want to pass from, in the perfect
    // position to just move forward a bit and take the kick
    Robot robot = Robot(13, Point(0, DIST_TO_FRONT_OF_ROBOT_METERS), Vector(),
                        Angle::fromDegrees(-90), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Robot should be offset from where the pass is supposed to start
    double robot_offset_meters =
        DIST_TO_FRONT_OF_ROBOT_METERS + 2 * BALL_MAX_RADIUS_METERS;

    // We're in the perfect position to kick, but pass hasn't started yet, so
    // we should just be moving
    auto move_action = std::dynamic_pointer_cast<MoveAction>(tactic.getNextAction());
    ASSERT_TRUE(move_action);
    EXPECT_TRUE(move_action->getRobot().has_value());
    EXPECT_EQ(13, move_action->getRobot()->id());
    EXPECT_NEAR(0, move_action->getDestination().x(), 1e-5);
    EXPECT_NEAR(robot_offset_meters, move_action->getDestination().y(), 1e-5);
    EXPECT_EQ(-90, move_action->getFinalOrientation().toDegrees());
    EXPECT_EQ(0, move_action->getFinalSpeed());
}

TEST(PasserTacticTest, passer_in_position_to_kick_pass_started)
{
    // Robot is sitting just behind where we want to pass from, in the perfect
    // position to take the kick
    Robot robot = Robot(13, Point(0, DIST_TO_FRONT_OF_ROBOT_METERS), Vector(),
                        Angle::fromDegrees(-90), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // Robots for creating Teams
    Robot enemy_robot(1, Point(0, 0), Vector(0, 0), Angle::half(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_robot(0, Point(-1, -1), Vector(0, 0), Angle::zero(),
                         AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // Teams for creating a world
    Team enemy_team    = Team({enemy_robot}, Duration::fromSeconds(1));
    Team friendly_team = Team({friendly_robot}, Duration::fromSeconds(1));

    // Ball not moving initially
    Ball ball({0, 0}, {0, 0}, Timestamp::fromSeconds(5));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // We should try to kick the ball
    auto kick_action = std::dynamic_pointer_cast<KickAction>(tactic.getNextAction());
    ASSERT_TRUE(kick_action);
    EXPECT_TRUE(kick_action->getRobot().has_value());
    EXPECT_EQ(13, kick_action->getRobot()->id());
    EXPECT_EQ(-90, kick_action->getKickDirection().toDegrees());
    EXPECT_EQ(Point(0, 0), kick_action->getKickOrigin());
    EXPECT_EQ(2.29, kick_action->getKickSpeed());

    // Ball starts moving as if we've kicked it
    ball = Ball({0, 0}, {0, -2}, Timestamp::fromSeconds(5.1));

    // World with updated ball and field
    World world = World(field, ball, friendly_team, enemy_team);
    tactic.updateWorldParams(world);
    tactic.updateControlParams(pass);

    // We need to try to get the next the intent to make the tactic finish
    tactic.getNextAction();

    // The tactic should now be done
    EXPECT_TRUE(tactic.done());
}

TEST(PasserTacticTest, kick_pass_started_with_moving_ball)
{
    // Robot is sitting just behind where we want to pass from, in the perfect
    // position to take the kick
    Robot robot = Robot(13, Point(0, 0), Vector(), Angle::fromDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(1));

    // Ball not moving initially
    Ball ball({0, 0.5}, {-2, 0}, Timestamp::fromSeconds(1));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(0));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Initially try intercept the ball to make sure we have control
    auto intercept_action =
        std::dynamic_pointer_cast<InterceptBallAction>(tactic.getNextAction());
    ASSERT_TRUE(intercept_action);
    EXPECT_TRUE(intercept_action->getRobot().has_value());
    EXPECT_EQ(13, intercept_action->getRobot()->id());
}

TEST(PasserTacticTest, kick_pass_not_yet_started_with_moving_ball)
{
    // Robot is sitting just behind where we want to pass from, in the perfect
    // position to take the kick
    Robot robot = Robot(13, Point(0, 0), Vector(), Angle::fromDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(1));

    // Ball not moving initially
    Ball ball({0, 0.5}, {-2, 0}, Timestamp::fromSeconds(1));
    Field field = Field::createSSLDivisionBField();

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(5));
    PasserTactic tactic(pass, ball, field, false);

    tactic.updateRobot(robot);

    // Initially try intercept the ball to make sure we have control
    auto intercept_action =
        std::dynamic_pointer_cast<InterceptBallAction>(tactic.getNextAction());
    ASSERT_TRUE(intercept_action);
    EXPECT_TRUE(intercept_action->getRobot().has_value());
    EXPECT_EQ(13, intercept_action->getRobot()->id());
}
