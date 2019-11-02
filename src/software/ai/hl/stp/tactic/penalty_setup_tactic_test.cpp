#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(PenaltySetupTacticTest, constructor_test)
{
    PenaltySetupTactic tactic = PenaltySetupTactic(true);

    EXPECT_TRUE("Penalty Setup Tactic" == tactic.getName());
}
