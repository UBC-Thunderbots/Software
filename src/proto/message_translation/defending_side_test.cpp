#include "proto/message_translation/defending_side.h"

#include <gtest/gtest.h>

TEST(TeamSideMsgTest, defending_positive_side)
{
    auto defending_side_msg = createDefendingSide(FieldSide::POS_X);
    EXPECT_EQ(DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_POS_X,
              defending_side_msg->defending_side());
}

TEST(TeamSideMsgTest, defending_negative_side)
{
    auto defending_side_msg = createDefendingSide(FieldSide::NEG_X);
    EXPECT_EQ(DefendingSideProto::FieldSide::DefendingSideProto_FieldSide_NEG_X,
              defending_side_msg->defending_side());
}
