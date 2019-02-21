/**
 * This file contains the unit tests for the Intent class
 */

#include "ai/intent/intent.h"

#include <gtest/gtest.h>

#include "ai/intent/move_intent.h"

TEST(IntentTest, test_get_priority)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle(), 0.0, 2);

    EXPECT_EQ(2, move_intent.getPriority());
}

TEST(IntentTest, test_set_priority)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle(), 0.0, 0);
    move_intent.setPriority(7);

    EXPECT_EQ(7, move_intent.getPriority());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
