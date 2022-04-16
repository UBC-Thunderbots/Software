#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <gtest/gtest.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(ShootOrPassPlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();

    FSM<ShootOrPassPlayFSM> fsm(
        ShootOrPassPlayFSM{std::make_shared<const ThunderbotsConfig>()->getAiConfig()});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(world, 3, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));
}

TEST(ShootOrPassPlayFSMTest, DISABLED_test_should_abort_pass_guard)
{
    World world = ::TestUtil::createBlankTestingWorld();
//    world.updateGameState(::TestUtil::createGameState(RefereeCommand::FORCE_START, RefereeCommand::HALT));

    FSM<ShootOrPassPlayFSM> fsm(
        ShootOrPassPlayFSM{std::make_shared<const ThunderbotsConfig>()->getAiConfig()});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));



    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(world, 2, [](PriorityTacticVector new_tactics) {})));
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));


    Robot friendly_robot_1(1, Point(0, 0), Vector(0, 0), Angle::half(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0.5));
    Robot friendly_robot_2(2, Point(2, 1), Vector(0, 0), Angle::zero(),
                      AngularVelocity::zero(), Timestamp::fromSeconds(0.5));
    std::vector<Robot> friendlies = {friendly_robot_1, friendly_robot_2};

    world.updateFriendlyTeamState(Team(friendlies));
    fsm.process_event(ShootOrPassPlayFSM::Update(
            ShootOrPassPlayFSM::ControlParams{},
            PlayUpdate(world, 2, [](PriorityTacticVector new_tactics) {})));
//    std::cout << getCurrentFullStateName(fsm) << std::endl;



    world.updateBall(Ball(Point(0.25,0), Vector(0, 0), Timestamp::fromSeconds(0)));
    world.setTeamWithPossession(TeamSide::FRIENDLY);



    fsm.process_event(ShootOrPassPlayFSM::Update(
            ShootOrPassPlayFSM::ControlParams{},
            PlayUpdate(world, 2, [](PriorityTacticVector new_tactics) {})));

    std::this_thread::sleep_for(std::chrono::milliseconds(300));



//    std::cout << getCurrentFullStateName(fsm) << std::endl;
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::TakePassState>));


    Robot enemy_interceptor(1, Point(2, 0), Vector(-0.5, 0), Angle::half(),
                           AngularVelocity::zero(), Timestamp());

    std::vector<Robot> enemy_team = {enemy_interceptor};
//    world.updateEnemyTeamState(Team(enemy_team));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(world, 2, [](PriorityTacticVector new_tactics) {})));


    // fsm is in the take pass state, and the friendly robot is about to pass
    // interceptor gets in the way of the pass, so friendly should abort the pass
    std::cout << getCurrentFullStateName(fsm) << std::endl;
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::TakePassState>));




}

TEST(ShootOrPassPlayFSMTest, test_took_shot_guard) {
    World world = ::TestUtil::createBlankTestingWorld();

    FSM<ShootOrPassPlayFSM> fsm(
            ShootOrPassPlayFSM{std::make_shared<const ThunderbotsConfig>()->getAiConfig()});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
            ShootOrPassPlayFSM::ControlParams{},
            PlayUpdate(world, 3, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));


    Robot friendly_robot_1(1, Point(5, 0), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero(), Timestamp());
    std::vector<Robot> friendlies = {friendly_robot_1};
    world.updateFriendlyTeamState(Team(friendlies));
    world.updateBall(Ball(Point(5.0,0), Vector(10,0), Timestamp::fromSeconds(1)));
    world.setTeamWithPossession(TeamSide::FRIENDLY);

    fsm.process_event(ShootOrPassPlayFSM::Update(
            ShootOrPassPlayFSM::ControlParams{},
            PlayUpdate(world, 3, [](PriorityTacticVector new_tactics) {})));
//    std::cout << getCurrentFullStateName(fsm) << std::endl;

    // friendly robot is in front of goal, no other robots to pass to,
    // he takes the shot and triggers the tookShot guard, fsm goes into termination state
    EXPECT_TRUE(fsm.is(boost::sml::state<boost::sml::back::terminate_state>));
}
