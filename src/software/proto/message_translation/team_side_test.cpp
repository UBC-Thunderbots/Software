#include "software/proto/message_translation/team_side.h"

#include <gtest/gtest.h>

TEST(TeamSideMsgTest, create_msg_with_true_value)
{
    auto team_side_msg = createTeamSideMsg(true);
    EXPECT_TRUE(team_side_msg->defending_positive_side());
}

TEST(TeamSideMsgTest, create_msg_with_false_value)
{
    auto team_side_msg = createTeamSideMsg(false);
    EXPECT_FALSE(team_side_msg->defending_positive_side());
}

