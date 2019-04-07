/**
 * Tests for the Passer Tactic
 */
#include "ai/hl/stp/tactic/passer_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "test/test_util/test_util.h"

using namespace AI::Passing;

TEST(PasserTacticTest, passer_already_at_pass_start_position_but_oriented_incorrectly)
{
    // Robot is sitting at origin facing towards enemy goal
    Robot robot = Robot(13, Point(0, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(0));
    PasserTactic tactic(pass, false);

    tactic.updateRobot(robot);

    // In this case we should be moving into position to kick the ball, since we're
    // in front of it and need to get behind it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0, move_intent.getDestination().x(), 1e-5);
    EXPECT_LE(0, move_intent.getDestination().y());
    EXPECT_GE(0.5, move_intent.getDestination().y());
    EXPECT_EQ(-90, move_intent.getFinalAngle().toDegrees());
    EXPECT_EQ(0, move_intent.getFinalSpeed());
}

TEST(PasserTacticTest, passer_oriented_correctly_but_not_at_pass_start_position)
{
    // Robot is sitting at {1,2} facing towards -y
    Robot robot = Robot(13, Point(1, 2), Vector(), Angle::ofDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(0));
    PasserTactic tactic(pass, false);

    tactic.updateRobot(robot);

    // In this case we should be moving into position to kick the ball, since we're
    // in front of it and need to get behind it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0, move_intent.getDestination().x(), 1e-5);
    EXPECT_LE(0, move_intent.getDestination().y());
    EXPECT_GE(0.5, move_intent.getDestination().y());
    EXPECT_EQ(-90, move_intent.getFinalAngle().toDegrees());
    EXPECT_EQ(0, move_intent.getFinalSpeed());
}

TEST(PasserTacticTest,
     passer_oriented_incorrectly_and_close_to_start_position_but_in_front_of_pass)
{
    // Robot is sitting at {-0.3,0.2} facing towards -y
    Robot robot = Robot(13, Point(-0.3, 0.2), Vector(), Angle::ofDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We want to pass from the origin to 1 meter in the -y direction. This means the
    // robot needs to move around the ball to get toa point where it can kick
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(0));
    PasserTactic tactic(pass, false);

    tactic.updateRobot(robot);

    // In this case we should be moving into position to kick the ball, since we're
    // in front of it and need to get behind it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0, move_intent.getDestination().x(), 1e-5);
    EXPECT_LE(0, move_intent.getDestination().y());
    EXPECT_GE(0.5, move_intent.getDestination().y());
    EXPECT_EQ(-90, move_intent.getFinalAngle().toDegrees());
    EXPECT_EQ(0, move_intent.getFinalSpeed());
}

TEST(PasserTacticTest, passer_in_position_to_kick)
{
    // Robot is sitting just behind where we want to pass from, in the perfect
    // position to just move forward a bit and take the kick
    Robot robot = Robot(13, Point(0, 0.3), Vector(), Angle::ofDegrees(-90),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));

    // We want to pass from the origin to 1 meter in the -y direction
    Pass pass({0, 0}, {0, -1}, 2.29, Timestamp::fromSeconds(0));
    PasserTactic tactic(pass, false);

    tactic.updateRobot(robot);

    // Should yield one move intent and then stop
    KickIntent kick_intent = dynamic_cast<KickIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, kick_intent.getRobotId());
    EXPECT_EQ(-90, kick_intent.getKickDirection().toDegrees());
    EXPECT_EQ(Point(0, 0), kick_intent.getKickOrigin());
    EXPECT_EQ(2.29, kick_intent.getKickSpeed());
}
