#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(ShootOrPassPlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();

    TbotsProto::AiConfig ai_config;
    FSM<ShootOrPassPlayFSM> fsm(ShootOrPassPlayFSM{ai_config, std::make_shared<Strategy>(ai_config)});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));
}

TEST(ShootOrPassPlayFSMTest, test_abort_pass_guard)
{
    World world = ::TestUtil::createBlankTestingWorld();
    world.updateRefereeCommand(RefereeCommand::FORCE_START);

    TbotsProto::AiConfig ai_config;
    FSM<ShootOrPassPlayFSM> fsm(ShootOrPassPlayFSM{ai_config, std::make_shared<Strategy>(ai_config)});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 4, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));

    world.updateBall(Ball(Point(-1, 0), Vector(0, 0), Timestamp::fromSeconds(1)));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 2, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    Robot friendly_robot_1(1, Point(3, -1), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(2));
    Robot friendly_robot_2(2, Point(0, 0), Vector(0, 0), Angle::half(),
                           AngularVelocity::zero(), Timestamp::fromSeconds(2));
    std::vector<Robot> friendlies = {friendly_robot_1, friendly_robot_2};
    world.updateFriendlyTeamState(Team(friendlies));

    // have the fsm process an updated world
    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 2, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    // 2 process events needed so that fsm finds a pass between the 2 robots on the field.
    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::TakePassState>));

    world.updateBall(Ball(Point(1, 0), Vector(0, 0), Timestamp::fromSeconds(3)));

    // ball moved, so we should abort the pass, and transition back into attempt shot
    // state
    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 2, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));
}


TEST(ShootOrPassPlayFSMTest, test_took_shot_guard)
{
    World world = ::TestUtil::createBlankTestingWorld();

    TbotsProto::AiConfig ai_config;
    FSM<ShootOrPassPlayFSM> fsm(ShootOrPassPlayFSM{ai_config, std::make_shared<Strategy>(ai_config)});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));


    Robot friendly_robot_1(1, Point(2, 0), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero(), Timestamp());
    std::vector<Robot> friendlies = {friendly_robot_1};
    world.updateFriendlyTeamState(Team(friendlies));
    world.updateBall(Ball(Point(2.0, 0), Vector(10, 0), Timestamp::fromSeconds(1)));

    // have the fsm process an event with updated world
    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    // friendly robot is in front of goal, no other robots to pass to,
    // he takes the shot and triggers the tookShot guard, fsm goes into termination state
    EXPECT_TRUE(fsm.is(boost::sml::state<boost::sml::back::terminate_state>));
}
