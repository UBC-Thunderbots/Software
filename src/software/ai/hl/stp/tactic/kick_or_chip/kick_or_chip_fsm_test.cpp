#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(KickOrChipFSMTest, test_transitions_autokick)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickOrChipFSM::ControlParams control_params{
        .kick_or_chip_origin    = Point(-2, 1.5),
        .kick_or_chip_direction = Angle::threeQuarter(),
        .auto_chip_or_kick      = {AutoChipOrKickMode::AUTOKICK, 1.2}};

    FSM<KickOrChipFSM> fsm{KickOrChipFSM(std::make_shared<TbotsProto::AiConfig>()),
                           GetBehindBallFSM(std::make_shared<TbotsProto::AiConfig>())};

    // Start in GetBehindBallFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Transition to GetBehindBallFSM state's GetBehindBallState
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Change the kick direction and expect the FSM to realign
    control_params.kick_or_chip_direction = Angle::quarter();
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball in the new direction
    robot = Robot(
        0,
        RobotState(Point(-2, 1.3), Vector(), Angle::quarter(), AngularVelocity::zero()),
        Timestamp::fromSeconds(124));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState again
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Ball is now kicked
    ::TestUtil::setBallVelocity(world, Vector(0, 2.1), Timestamp::fromSeconds(123));

    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::quarter()));

    // Tactic is done
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}

TEST(KickOrChipFSMTest, test_transitions_autochip)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    Robot robot                  = ::TestUtil::createRobotAtPos(Point(-2, -3));
    KickOrChipFSM::ControlParams control_params{
        .kick_or_chip_origin    = Point(-2, 1.5),
        .kick_or_chip_direction = Angle::threeQuarter(),
        .auto_chip_or_kick      = {AutoChipOrKickMode::AUTOCHIP, 1.2}};

    FSM<KickOrChipFSM> fsm{KickOrChipFSM(std::make_shared<TbotsProto::AiConfig>()),
                           GetBehindBallFSM(std::make_shared<TbotsProto::AiConfig>())};

    // Start in GetBehindBallFSM state's GetBehindBallState
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Transition to GetBehindBallFSM state's GetBehindBallState
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball
    robot = Robot(0,
                  RobotState(Point(-2, 1.7), Vector(), Angle::threeQuarter(),
                             AngularVelocity::zero()),
                  Timestamp::fromSeconds(123));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Change the chip direction and expect the FSM to realign
    control_params.kick_or_chip_direction = Angle::quarter();
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<GetBehindBallFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<GetBehindBallFSM>)>(
        boost::sml::state<GetBehindBallFSM::GetBehindBallState>));

    // Robot is now behind ball in the new direction
    robot = Robot(
        0,
        RobotState(Point(-2, 1.3), Vector(), Angle::quarter(), AngularVelocity::zero()),
        Timestamp::fromSeconds(124));
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    // Transition to KickOrChipState again
    EXPECT_TRUE(fsm.is(boost::sml::state<KickOrChipFSM::KickOrChipState>));

    // Ball is now chipped
    ::TestUtil::setBallVelocity(world, Vector(0, 2.1), Timestamp::fromSeconds(123));

    EXPECT_TRUE(world->ball().hasBallBeenKicked(Angle::quarter()));

    // Tactic is done
    fsm.process_event(KickOrChipFSM::Update(
        control_params, TacticUpdate(robot, world, [](std::shared_ptr<Primitive>) {})));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
