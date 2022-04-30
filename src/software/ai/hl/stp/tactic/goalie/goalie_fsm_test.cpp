#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_get_goalie_position_to_block)
{
    Field field = Field::createSSLDivisionBField();
    std::shared_ptr<GoalieTacticConfig> goalie_tactic_config =
        std::make_shared<GoalieTacticConfig>();

    // ball at center field, goalie should position itself at the front of the friendly
    // defense area
    Ball ball = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(123));
    Point goalie_pos =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos));
    EXPECT_EQ(Point(field.friendlyDefenseArea().xMax(), 0), goalie_pos);

    // ball at positive friendly corner, goalie should snap to positive goal post
    ball.updateState(BallState(field.friendlyCornerPos(), Vector(0, 0)),
                     Timestamp::fromSeconds(123));
    Point goalie_pos_corner_positive =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos_corner_positive));
    EXPECT_EQ(field.friendlyGoalpostPos() + Vector(0, -ROBOT_MAX_RADIUS_METERS),
              goalie_pos_corner_positive);

    // ball at negative friendly corner, goalie should snap to negative goal post
    ball.updateState(BallState(field.friendlyGoalpostNeg(), Vector(0, 0)),
                     Timestamp::fromSeconds(123));
    Point goalie_pos_corner_negative =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos_corner_negative));
    EXPECT_EQ(field.friendlyGoalpostNeg() + Vector(0, ROBOT_MAX_RADIUS_METERS),
              goalie_pos_corner_negative);

    // ball in friendly defense area
    ball.updateState(BallState(Point(-4, 0), Vector(0, 0)), Timestamp::fromSeconds(123));
    EXPECT_TRUE(
        contains(field.friendlyDefenseArea(),
                 GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config)));

    // ball on positive-y side of field
    ball.updateState(BallState(Point(-4, 2), Vector(0, 0)), Timestamp::fromSeconds(123));
    EXPECT_TRUE(
        contains(field.friendlyDefenseArea(),
                 GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config)));

    // ball on negative-y side of field
    ball.updateState(BallState(Point(-4, -2), Vector(0, 0)), Timestamp::fromSeconds(123));
    EXPECT_TRUE(
        contains(field.friendlyDefenseArea(),
                 GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config)));
}

TEST(GoalieFSMTest, test_get_intersections_between_ball_velocity_and_full_goal_segment)
{
    Field field = Field::createSSLDivisionBField();

    // ball has intersection with friendly goal
    Ball ball = Ball(Point(0, 0), Vector(-1, 0), Timestamp::fromSeconds(123));
    std::vector<Point> intersections =
        GoalieFSM::getIntersectionsBetweenBallVelocityAndFullGoalSegment(ball, field);
    EXPECT_TRUE(contains(field.friendlyGoal(), intersections[0]));
    EXPECT_EQ(field.friendlyGoalCenter(), intersections[0]);

    // ball has no intersection with friendly goal
    ball.updateState(BallState(Point(0, 0), Vector(1, 0)), Timestamp::fromSeconds(123));
    intersections =
        GoalieFSM::getIntersectionsBetweenBallVelocityAndFullGoalSegment(ball, field);
    EXPECT_TRUE(intersections.empty());
}

TEST(GoalieFSMTest, test_transitions)
{
    Robot goalie = ::TestUtil::createRobotAtPos(Point(-4.5, 0));
    World world  = ::TestUtil::createBlankTestingWorld();

    world = ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));
    Point clear_ball_origin =
        Point(GoalieFSM::getNoChipRectangle(world.field()).xMax(), 0);
    Angle clear_ball_direction = Angle::zero();

    FSM<GoalieFSM> fsm(DribbleFSM(std::make_shared<DribbleTacticConfig>()),
                       GoalieFSM(std::make_shared<const GoalieTacticConfig>(),
                                 TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT));

    // goalie starts in PositionToBlock
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving slowly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));

    // goalie should remain in PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));

    // goalie should transition to Panic
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::Panic>));

    // ball is now out of danger
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));

    // process event again to reset goalie to PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now stationary in the "no-chip" rectangle
    world = ::TestUtil::setBallPosition(world, world.field().friendlyGoalCenter(),
                                        Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));

    // goalie should transition to DribbleFSM
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));

    // goalie has ball, at the correct position and orientation to clear the ball
    world = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    goalie.updateState(RobotState(clear_ball_origin, Vector(0, 0), clear_ball_direction,
                                  AngularVelocity::zero()),
                       Timestamp::fromSeconds(123));

    // goalie should stay in PivotKickFSM but be ready to chip
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));

    goalie = ::TestUtil::createRobotAtPos(clear_ball_origin + Vector(-0.2, 0));
    world  = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    // ball is now chipped
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(clear_ball_direction));

    // ball is out of defense area
    world = ::TestUtil::setBallPosition(world, Point(-2, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));

    // process event once to reset goalie to PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving slowly inside the friendly defense area
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(124));

    // goalie should transition to PivotKickFSM
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(
                goalie, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(124));
}
