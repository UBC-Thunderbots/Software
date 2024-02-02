#include "software/ai/hl/stp/tactic/move/move_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(MoveFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    MoveFSM::ControlParams control_params{
        .destination            = Point(2, 3),
        .final_orientation      = Angle::half(),
        .final_speed            = 0.0,
        .dribbler_mode          = TbotsProto::DribblerMode::OFF,
        .ball_collision_type    = TbotsProto::BallCollisionType::AVOID,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        .target_spin_rev_per_s  = 0.0};

    FSM<MoveFSM> fsm;
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::MoveState>));

    // robot far from destination
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::MoveState>));

    // robot close to destination
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::MoveState>));

    // robot at destination and facing the right way
    robot.updateState(
        RobotState(Point(2, 3), Vector(), Angle::half(), AngularVelocity::zero()),
        Timestamp::fromSeconds(0));
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // destination updated so robot needs to move to new destination
    control_params = MoveFSM::ControlParams{
        .destination            = Point(1, -3),
        .final_orientation      = Angle::half(),
        .final_speed            = 0.0,
        .dribbler_mode          = TbotsProto::DribblerMode::OFF,
        .ball_collision_type    = TbotsProto::BallCollisionType::AVOID,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        .target_spin_rev_per_s  = 0.0};
    fsm.process_event(MoveFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveFSM::MoveState>));
}
