#include "software/ai/hl/stp/play/shoot_or_pass/shoot_or_pass_play_fsm.h"

#include <gtest/gtest.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(ShootOrPassPlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();

    TbotsProto::AiConfig ai_config;
    FSM<ShootOrPassPlayFSM> fsm(ShootOrPassPlayFSM{ai_config});
    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::StartState>));

    fsm.process_event(ShootOrPassPlayFSM::Update(
        ShootOrPassPlayFSM::ControlParams{},
        PlayUpdate(world, 3, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::state<ShootOrPassPlayFSM::AttemptShotState>));
}
