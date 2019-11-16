#include "software/ai/hl/stp/tactic/tactic_update_visitor.h"

#include <gtest/gtest.h>

#include "software/geom/util.h"
#include "software/test_util/test_util.h"

TEST(TacticUpdateVisitorTest, update_cherry_pick_tactic)
{
    World world = ::Test::TestUtil::createBlankTestingWorld();
    CherryPickTactic tactic = CherryPickTactic(world, rectangle);
}