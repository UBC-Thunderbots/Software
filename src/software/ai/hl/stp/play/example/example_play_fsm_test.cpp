#include "software/ai/hl/stp/play/example/example_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(ExamplePlayFSMTest, test_transitions)
{
    std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();

    FSM<ExamplePlayFSM> fsm(ExamplePlayFSM{std::make_shared<TbotsProto::AiConfig>()});

    EXPECT_TRUE(fsm.is(boost::sml::state<ExamplePlayFSM::MoveState>));

    int num_tactics = 6;

    fsm.process_event(ExamplePlayFSM::Update(
        ExamplePlayFSM::ControlParams{},
        PlayUpdate(
            world, num_tactics, [](PriorityTacticVector new_tactics) {},
            InterPlayCommunication{}, [](InterPlayCommunication comm) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ExamplePlayFSM::MoveState>));
}
