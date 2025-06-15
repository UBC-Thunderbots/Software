#include "software/ai/hl/stp/play/defense/defense_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(DefensePlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    TbotsProto::AiConfig ai_config;

    FSM<DefensePlayFSM> fsm(DefensePlayFSM{ai_config});
    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::DefenseState>));

    //Place enemy robots behind the ball implying there is a large attack
    ::TestUtil::setEnemyRobotPositions(world, {Point(0, 0), Point(-1, 0), Point(-2, 0)},
                                          Timestamp::fromSeconds(0));

    fsm.process_event(DefensePlayFSM::Update(
        DefensePlayFSM::ControlParams{
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::DefenseState>));
    

    //Place more friendly robots behind the ball implying this is a safe time
    //to aggressively defense
    ::TestUtil::setFriendlyRobotPositions(world, {Point(-1, 0), Point(-3, 0), Point(-2, 0)},
                                          Timestamp::fromSeconds(0));

    fsm.process_event(DefensePlayFSM::Update(
        DefensePlayFSM::ControlParams{
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));
    
    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::AggressiveDefenseState>));
}
