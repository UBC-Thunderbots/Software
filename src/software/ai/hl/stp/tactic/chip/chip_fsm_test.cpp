#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ChipFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    ChipFSM::ControlParams control_params{.chip_origin          = Point(-2, 1.5),
                                          .chip_direction       = Angle::threeQuarter(),
                                          .chip_distance_meters = 1.2};

    FSM<ChipFSM> fsm{GetBehindBallFSM()};

    // Start in GetBehindBallFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Transition to GetBehindBallFSM state's GetBehindBallState
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to ChipState
    EXPECT_TRUE(fsm.is(boost::sml::state<ChipFSM::ChipState>));

    // Ball is now chipped
    ::TestUtil::setBallVelocity(world, Vector(0, -2.1), Timestamp::fromSeconds(123));

    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::threeQuarter()));

    // Tactic is done
    fsm.process_event(ChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
