#include "software/ai/hl/stp/skill/chip/chip_skill_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ChipSkillFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    ChipSkillFSM::ControlParams control_params{.chip_origin          = Point(-2, 1.5),
                                          .chip_direction       = Angle::threeQuarter(),
                                          .chip_distance_meters = 1.2};

    FSM<ChipSkillFSM> fsm{GetBehindBallSkillFSM()};

    // Start in GetBehindBallSkillFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallSkillFSM>)>(
        boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // Transition to GetBehindBallSkillFSM state's GetBehindBallState
    fsm.process_event(ChipSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallSkillFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallSkillFSM>)>(
        boost::sml::state<GetBehindBallSkillFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(ChipSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to ChipState
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipSkillFSM::ChipState>));

    // Ball is now chipped
    ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));

    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(ChipSkillFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
