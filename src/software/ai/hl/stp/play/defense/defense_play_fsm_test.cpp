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

    fsm.process_event(DefensePlayFSM::Update(
        DefensePlayFSM::ControlParams{
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    // DefensePlayFSM always stays in the DefenseState
    EXPECT_TRUE(fsm.is(boost::sml::state<DefensePlayFSM::DefenseState>));
}
