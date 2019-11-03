#include "software/ai/hl/stp/tactic/chip_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ChipTacticTest, constructor_test)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    world =
        ::Test::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));

    ChipTactic tactic = ChipTactic(world.ball(), true);

    EXPECT_TRUE("Chip Tactic" == tactic.getName());
}
