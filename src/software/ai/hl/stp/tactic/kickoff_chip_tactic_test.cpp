#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(KickoffChipTacticTest, constructor_test)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    KickoffChipTactic tactic = KickoffChipTactic(world.ball(), true);

    EXPECT_TRUE("Kickoff Chip Tactic" == tactic.getName());
}
