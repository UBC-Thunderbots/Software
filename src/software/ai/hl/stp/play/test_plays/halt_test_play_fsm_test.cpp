#include "software/ai/hl/stp/play/test_plays/halt_test_play_fsm.h"

#include <gtest/gtest.h>

#include "proto/parameters.pb.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

  TEST(HaltTestPlayFSMTest, test_transitions)
  {
      std::shared_ptr<World> world = ::TestUtil::createBlankTestingWorld();
      FSM<HaltTestPlayFSM> fsm(HaltTestPlayFSM{std::make_shared<TbotsProto::AiConfig>()});

      // Verify initial state
      EXPECT_TRUE(fsm.is(boost::sml::state<HaltTestPlayFSM::HaltTestState>));

      // Capture tactics from callback
      PriorityTacticVector received_tactics;
      fsm.process_event(HaltTestPlayFSM::Update(
          HaltTestPlayFSM::ControlParams{},
          PlayUpdate(
              world, 3,
              [&received_tactics](PriorityTacticVector new_tactics) {
                  received_tactics = std::move(new_tactics);
              },
              InterPlayCommunication{}, [](InterPlayCommunication) {})));

      // Verify state stays the same (single-state loop)
      EXPECT_TRUE(fsm.is(boost::sml::state<HaltTestPlayFSM::HaltTestState>));
  }
