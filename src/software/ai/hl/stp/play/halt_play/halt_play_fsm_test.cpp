#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(HaltPlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    TbotsProto::AiConfig ai_config;

    FSM<HaltPlayFSM> fsm(HaltPlayFSM{ai_config});

    EXPECT_TRUE(fsm.is(boost::sml::state<HaltPlayFSM::HaltState>));

    int num_tactics = 6;

    fsm.process_event(HaltPlayFSM::Update(
        HaltPlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<HaltPlayFSM::HaltState>));
}
