#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefensePlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    TbotsProto::AiConfig ai_config;

    FSM<CreaseDefensePlayFSM> fsm(CreaseDefensePlayFSM{ai_config});
    EXPECT_TRUE(fsm.is(boost::sml::state<CreaseDefensePlayFSM::DefenseState>));

    fsm.process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin    = Point(),
            .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(
            world, 3, [](PriorityTacticVector new_tactics) {}, InterPlayCommunication{},
            [](InterPlayCommunication comm) {})));

    // CreaseDefensePlayFSM always stays in the DefenseState
    EXPECT_TRUE(fsm.is(boost::sml::state<CreaseDefensePlayFSM::DefenseState>));
}
