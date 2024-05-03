#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/test_util/test_util.h"

TEST(GetBehindBallSkillFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    GetBehindBallSkillFSM::ControlParams control_params{
        .ball_location = Point(2, 3), .chick_direction = Angle::quarter()};

    FSM<GetBehindBallSkillFSM> fsm{GetBehindBallSkillFSM()};
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));
    fsm.process_event(GetBehindBallSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // robot behind ball but far away
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(GetBehindBallSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // robot behind ball and close enough
    robot = Robot(
        0, RobotState(Point(2, 2.8), Vector(), Angle::quarter(), AngularVelocity::zero()),
        Timestamp::fromSeconds(123));
    fsm.process_event(GetBehindBallSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));

    // destination updated so robot needs to move to new destination
    control_params = GetBehindBallSkillFSM::ControlParams{
        .ball_location = Point(-2, 1), .chick_direction = Angle::quarter()};
    fsm.process_event(GetBehindBallSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));
}
