#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    MoveFSM::ControlParams control_params{
        .destination            = Point(2, 3),
        .final_orientation      = Angle::half(),
        .final_speed            = 0.0,
        .dribbler_mode          = DribblerMode::OFF,
        .ball_collision_type    = BallCollisionType::AVOID,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT};

    BaseFSM<MoveFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));

    // robot far from destination
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));

    // robot close to destination
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));

    // robot at destination and facing the right way
    robot.updateState(
        RobotState(Point(2, 3), Vector(), Angle::half(), AngularVelocity::zero()),
        Timestamp::fromSeconds(0));
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // destination updated so robot needs to move to new destination
    control_params = MoveFSM::ControlParams{
        .destination            = Point(1, -3),
        .final_orientation      = Angle::half(),
        .final_speed            = 0.0,
        .dribbler_mode          = DribblerMode::OFF,
        .ball_collision_type    = BallCollisionType::AVOID,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT};
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::unique_ptr<Intent>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::move_state>));
}
