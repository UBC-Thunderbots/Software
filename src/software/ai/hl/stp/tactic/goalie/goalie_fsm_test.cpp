#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

TEST(GoalieFSMTest, test_get_goalie_position_to_block)
{
    Field field = Field::createSSLDivisionBField();
    TbotsProto::GoalieTacticConfig goalie_tactic_config;

    // ball at center field, goalie should position itself at conservative depth
    // in line with the ball
    Ball ball = Ball(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(123));
    Point goalie_pos =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos));
    EXPECT_EQ(Point(field.friendlyDefenseArea().xMin() +
                        goalie_tactic_config.conservative_depth_meters(),
                    0),
              goalie_pos);

    // ball at positive friendly corner, goalie should snap to positive goal post
    ball.updateState(BallState(field.friendlyCornerPos(), Vector(0, 0)),
                     Timestamp::fromSeconds(123));
    Point goalie_pos_corner_positive =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos_corner_positive));
    EXPECT_EQ(field.friendlyGoalpostPos() + Vector(ROBOT_MAX_RADIUS_METERS, 0),
              goalie_pos_corner_positive);

    // ball at negative friendly corner, goalie should snap to negative goal post
    ball.updateState(BallState(field.friendlyGoalpostNeg(), Vector(0, 0)),
                     Timestamp::fromSeconds(123));
    Point goalie_pos_corner_negative =
        GoalieFSM::getGoaliePositionToBlock(ball, field, goalie_tactic_config);
    EXPECT_TRUE(contains(field.friendlyDefenseArea(), goalie_pos_corner_negative));
    EXPECT_EQ(field.friendlyGoalpostNeg() + Vector(ROBOT_MAX_RADIUS_METERS, 0),
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

    TbotsProto::AiConfig ai_config;
    FSM<GoalieFSM> fsm(DribbleSkillFSM(ai_config.dribble_skill_config()),
                       GoalieFSM(ai_config.goalie_tactic_config(),
                                 TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT));

    // goalie starts in PositionToBlock
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving slowly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-0.1, 0), Timestamp::fromSeconds(123));

    // goalie should remain in PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallVelocity(world, Vector(-1, 0), Timestamp::fromSeconds(123));

    // goalie should transition to Panic
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::Panic>));

    // ball is now out of danger
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));

    // process event again to reset goalie to PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now stationary in the "no-chip" rectangle
    world = ::TestUtil::setBallPosition(world, world.field().friendlyGoalCenter(),
                                        Timestamp::fromSeconds(123));
    world = ::TestUtil::setBallVelocity(world, Vector(0, 0), Timestamp::fromSeconds(123));

    // goalie should transition to DribbleSkillFSM
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickSkillFSM>));

    // goalie has ball, at the correct position and orientation to clear the ball
    world = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    goalie.updateState(RobotState(clear_ball_origin, Vector(0, 0), clear_ball_direction,
                                  AngularVelocity::zero()),
                       Timestamp::fromSeconds(123));

    // goalie should stay in PivotKickSkillFSM but be ready to chip
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickSkillFSM>));

    goalie = ::TestUtil::createRobotAtPos(clear_ball_origin + Vector(-0.2, 0));
    world  = ::TestUtil::setBallPosition(world, clear_ball_origin,
                                        Timestamp::fromSeconds(123));
    // ball is now chipped
    world = ::TestUtil::setBallVelocity(world, Vector(1, 0), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(clear_ball_direction));

    // ball is out of defense area
    world = ::TestUtil::setBallPosition(world, Point(-2, 0), Timestamp::fromSeconds(123));
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));

    // process event once to reset goalie to PositionToBlock
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GoalieFSM::PositionToBlock>));

    // ball is now moving slowly inside the friendly defense area
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(0, -0.1), Timestamp::fromSeconds(124));

    // goalie should transition to PivotKickSkillFSM
    fsm.process_event(GoalieFSM::Update(
        {}, TacticUpdate(goalie, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickSkillFSM>));

    // ball is now moving quickly towards the friendly goal
    world =
        ::TestUtil::setBallPosition(world, Point(-3.5, 1), Timestamp::fromSeconds(124));
    world =
        ::TestUtil::setBallVelocity(world, Vector(-2, -1), Timestamp::fromSeconds(124));
}
