#include "software/proto/message_translation/defending_side.h"

#include <gtest/gtest.h>

TEST(TeamSideMsgTest, create_msg_with_true_value)
{
    auto defending_side_msg = createDefendingSideProto(true);
    EXPECT_TRUE(defending_side_msg->defending_positive_side());
}

TEST(TeamSideMsgTest, create_msg_with_false_value)
{
    auto defending_side_msg = createDefendingSideProto(false);
    EXPECT_FALSE(defending_side_msg->defending_positive_side());
}

