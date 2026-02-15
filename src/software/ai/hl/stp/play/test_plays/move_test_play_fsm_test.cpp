#include "software/ai/hl/stp/play/test_plays/move_test_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/test_util.h"

TEST(MoveTestPlayFSMTest, test_stays_in_move_state)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
    FSM<MoveTestPlayFSM> fsm(
        MoveTestPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    // Verify initial state
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveTestPlayFSM::MoveTestState>));

    // Capture tactics from callback
    PriorityTacticVector received_tactics;
    fsm.process_event(MoveTestPlayFSM::Update(
        MoveTestPlayFSM::ControlParams{},
        PlayUpdate(
            world, 3,
            [&received_tactics](PriorityTacticVector new_tactics)
            { received_tactics = std::move(new_tactics); },
            InterPlayCommunication{}, [](InterPlayCommunication) {})));

    // Verify state stays the same (tactic not done yet)
    EXPECT_TRUE(fsm.is(boost::sml::state<MoveTestPlayFSM::MoveTestState>));

    // Verify we got exactly 1 priority level with 3 tactics
    ASSERT_EQ(received_tactics.size(), 1);
    EXPECT_EQ(received_tactics[0].size(), 3);
}
