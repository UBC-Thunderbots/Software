#include "software/ai/hl/stp/action/kick_action.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/ai/intent/kick_intent.h"
#include "software/ai/intent/move_intent.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/test_util/test_util.h"

TEST(KickActionTest, getKickDirection)
{
    Robot robot(0, Point(-1, -2), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {-1, 2}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::quarter(), 5.0);

    EXPECT_EQ(Angle::quarter(), action.getKickDirection());
}

TEST(KickActionTest, getKickOrigin)
{
    Robot robot(0, Point(-1, -2), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {-1, 2}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, Point(10, -11), Angle::quarter(), 5.0);

    EXPECT_EQ(Point(10, -11), action.getKickOrigin());
}

TEST(KickActionTest, getKickSpeed)
{
    Robot robot(0, Point(-1, -2), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {-1, 2}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, Point(10, -11), Angle::quarter(), 5.0);

    EXPECT_EQ(5.0, action.getKickSpeed());
}

TEST(KickActionTest, robot_behind_ball_kicking_towards_positive_x_positive_y)
{
    Robot robot(0, Point(-0.3, 0), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {0, 0}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::zero(), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotID());
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_origin(),
            *createPointProto(Point(0, 0))));
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_direction(),
            *createAngleProto(Angle::zero())));
        EXPECT_EQ(5.0, kick_intent.getPrimitive().kick().kick_speed_meters_per_second());
    }
    catch (...)
    {
        ADD_FAILURE() << "Kick intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_behind_ball_kicking_towards_negative_x_positive_y)
{
    Robot robot(0, Point(-2.4, 2.3), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, {-2.5, 2.5}, Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(105),
                               5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotID());
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_origin(),
            *createPointProto(Point(-2.5, 2.5))));
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_direction(),
            *createAngleProto(Angle::fromDegrees(105))));
        EXPECT_EQ(5.0, kick_intent.getPrimitive().kick().kick_speed_meters_per_second());
    }
    catch (...)
    {
        ADD_FAILURE() << "Kick intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_behind_ball_kicking_towards_negative_x_negative_y)
{
    Robot robot(0, Point(0, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, {-0.05, -0.2}, Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(255),
                               3.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotID());
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_origin(),
            *createPointProto(Point(-0.05, -0.2))));
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_direction(),
            *createAngleProto(Angle::fromDegrees(255))));
        EXPECT_EQ(3.0, kick_intent.getPrimitive().kick().kick_speed_meters_per_second());
    }
    catch (...)
    {
        ADD_FAILURE() << "Kick intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_behind_ball_kicking_towards_positive_x_negative_y)
{
    Robot robot(0, Point(-0.125, 0.25), Vector(), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {0, 0}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(306),
                               5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        KickIntent kick_intent = dynamic_cast<KickIntent &>(*intent_ptr);
        EXPECT_EQ(0, kick_intent.getRobotID());
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_origin(),
            *createPointProto(Point(0, 0))));
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            kick_intent.getPrimitive().kick().kick_direction(),
            *createAngleProto(Angle::fromDegrees(306))));
        EXPECT_EQ(5.0, kick_intent.getPrimitive().kick().kick_speed_meters_per_second());
    }
    catch (...)
    {
        ADD_FAILURE() << "Kick intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_not_behind_ball_kicking_towards_positive_x_positive_y)
{
    Robot robot(0, Point(-1, 0.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {0, 0}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::zero(), 5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotID());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.18, 0), 0.1));
        EXPECT_EQ(Angle::zero(), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_not_behind_ball_kicking_towards_negative_x_positive_y)
{
    Robot robot(0, Point(-2, 5), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world = ::TestUtil::setBallPosition(world, {-2.5, 2.5}, Timestamp::fromSeconds(0));
    world = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(105),
                               5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotID());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-2.45, 2.25), 0.1));
        EXPECT_EQ(Angle::fromDegrees(105), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_not_behind_ball_kicking_towards_negative_x_negative_y)
{
    Robot robot(0, Point(0, 0), Vector(), Angle::quarter(), AngularVelocity::zero(),
                Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {-1, -4}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(255),
                               3.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotID());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.85, -3.75), 0.1));
        EXPECT_EQ(Angle::fromDegrees(255), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Kick Action!";
    }
}

TEST(KickActionTest, robot_not_behind_ball_kicking_towards_positive_x_negative_y)
{
    Robot robot(0, Point(0.5, 1), Vector(), Angle::threeQuarter(),
                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    World world = ::TestUtil::createBlankTestingWorld();
    world       = ::TestUtil::setBallPosition(world, {0, 0}, Timestamp::fromSeconds(0));
    world       = ::TestUtil::setBallVelocity(world, robot.position().toVector(),
                                        Timestamp::fromSeconds(0));

    KickAction action = KickAction();

    action.updateWorldParams(world);
    action.updateControlParams(robot, world.ball().position(), Angle::fromDegrees(306),
                               5.0);
    auto intent_ptr = action.getNextIntent();

    // Check an intent was returned (the pointer is not null)
    EXPECT_TRUE(intent_ptr);
    EXPECT_FALSE(action.done());

    try
    {
        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*intent_ptr);
        EXPECT_EQ(0, move_intent.getRobotID());
        // Check the MoveIntent is moving roughly behind the ball
        EXPECT_TRUE(TestUtil::equalWithinTolerance(move_intent.getDestination(),
                                                   Point(-0.15, 0.25), 0.1));
        EXPECT_EQ(Angle::fromDegrees(306), move_intent.getFinalAngle());
        EXPECT_EQ(0.0, move_intent.getFinalSpeed());
    }
    catch (...)
    {
        ADD_FAILURE() << "Move intent not returned by Kick Action!";
    }
}
