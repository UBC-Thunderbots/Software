#include "software/ai/hl/stp/skill/kick/kick_skill_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(KickSkillFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickSkillFSM::ControlParams control_params{.kick_origin    = Point(-2, 1.5),
                                               .kick_direction = Angle::threeQuarter(),
                                               .kick_speed_meters_per_second = 1.2};

    FSM<KickSkillFSM> fsm{GetBehindBallSkillFSM()};

    // Start in GetBehindBallSkillFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallSkillFSM>)>(
        boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // Transition to GetBehindBallSkillFSM state's GetBehindBallState
    fsm.process_event(KickSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallSkillFSM>)>(
        boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(KickSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickState
    EXPECT_TRUE(fsm.is(boost::sml::state<KickSkillFSM::KickState>));

    // Ball is now kicked
    ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));
    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(KickSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
