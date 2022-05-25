#include "software/ai/hl/stp/tactic/attacker/attacker_fsm.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(AttackerFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot = ::TestUtil::createRobotAtPos(Point(-2, -3));
    Pass pass   = Pass(Point(0, 0), Point(2, 0), 5);

    AttackerFSM::ControlParams control_params{.best_pass_so_far = pass,
                                              .pass_committed   = true,
                                              .shot             = std::nullopt,
                                              .chip_target      = std::nullopt};

    TbotsProto::AiConfig ai_config;
    FSM<AttackerFSM> fsm{DribbleFSM(ai_config.dribble_tactic_config()),
                         AttackerFSM(ai_config.attacker_tactic_config())};
    EXPECT_TRUE(fsm.is(boost::sml::state<DribbleFSM>));

    // robot far from attacker point
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));
    EXPECT_TRUE(
        fsm.is<decltype(boost::sml::state<PivotKickFSM>)>(boost::sml::state<DribbleFSM>));

    // robot close to attacker point
    robot = ::TestUtil::createRobotAtPos(Point(2, 2));
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));
    EXPECT_TRUE(
        fsm.is<decltype(boost::sml::state<PivotKickFSM>)>(boost::sml::state<DribbleFSM>));

    // robot at attacker point and facing the right way
    robot.updateState(RobotState(pass.passerPoint() - Vector(0.05, 0), Vector(),
                                 pass.passerOrientation(), AngularVelocity::zero()),
                      Timestamp::fromSeconds(0));

    // process event once to fall through the Dribble FSM
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));
    EXPECT_TRUE(
        fsm.is<decltype(boost::sml::state<PivotKickFSM>)>(boost::sml::state<DribbleFSM>));

    // robot should now kick the ball
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::state<PivotKickFSM>));
    EXPECT_TRUE(fsm.is<decltype(boost::sml::state<PivotKickFSM>)>(
        boost::sml::state<PivotKickFSM::KickState>));

    // FSM should be done now after 2 ticks, multiple ticks are required due to the
    // subFSMs
    world = ::TestUtil::setBallVelocity(world, Vector(5, 0), Timestamp::fromSeconds(223));
    EXPECT_TRUE(world.ball().hasBallBeenKicked(pass.passerOrientation()));
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    fsm.process_event(AttackerFSM::Update(
        control_params, TacticUpdate(
                            robot, world, [](std::unique_ptr<TbotsProto::Primitive>) {},
                            TEST_UTIL_CREATE_MOTION_CONTROL_NO_DEST)));
    EXPECT_TRUE(fsm.is(boost::sml::X));
}
