#include "software/ai/hl/stp/play/crease_defense/crease_defense_play_fsm.h"

#include <gtest/gtest.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/test_util/test_util.h"

TEST(CreaseDefensePlayFSMTest, test_transitions)
{
    World world = ::TestUtil::createBlankTestingWorld();

    FSM<CreaseDefensePlayFSM> fsm(CreaseDefensePlayFSM{
        std::make_shared<const ThunderbotsConfig>()->getPlayConfig()});
    EXPECT_TRUE(fsm.is(boost::sml::X));

    fsm.process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin                     = Point(),
            .num_additional_crease_defenders_tactics = 2,
            .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(world, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));

    fsm.process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin                     = Point(),
            .num_additional_crease_defenders_tactics = 2,
            .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(world, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));

    // Change the number of defenders
    fsm.process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin                     = Point(),
            .num_additional_crease_defenders_tactics = 1,
            .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(world, [](PriorityTacticVector new_tactics) {})));

    // FSM goes back to terminal state to re-set up the defenders
    EXPECT_TRUE(fsm.is(boost::sml::X));

    fsm.process_event(CreaseDefensePlayFSM::Update(
        CreaseDefensePlayFSM::ControlParams{
            .enemy_threat_origin                     = Point(),
            .num_additional_crease_defenders_tactics = 1,
            .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT},
        PlayUpdate(world, [](PriorityTacticVector new_tactics) {})));

    EXPECT_TRUE(fsm.is(boost::sml::X));
}
