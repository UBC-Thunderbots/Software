#include "software/ai/passing/chip_pass.h"

#include <gtest/gtest.h>

TEST(PassTest, simple_getters)
{
    ChipPass p(Point(1, 2), Point(3, 4));

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    std::cout << p.firstBounceRange() << std::endl;
}