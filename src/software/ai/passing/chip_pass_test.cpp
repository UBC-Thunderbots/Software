#include "software/ai/passing/chip_pass.h"

#include <gtest/gtest.h>

TEST(PassTest, simple_getters)
{
    ChipPass p(Point(1, 2), Point(3, 4));

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    std::cout << p.firstBounceRange() << std::endl;
}

// TEST(PassTest2, simple_getters)
// {
//     ChipPass p(Point(0, 0), Point(2, 0));

//     EXPECT_EQ(Point(0, 0), p.passerPoint());
//     EXPECT_EQ(Point(2, 0), p.receiverPoint());
//     std::cout << p.firstBounceRange() << std::endl;
// }

// TEST(PassTest3, simple_getters)
// {
//     ChipPass p(Point(0, 0), Point(3, 0));

//     EXPECT_EQ(Point(0, 0), p.passerPoint());
//     EXPECT_EQ(Point(3, 0), p.receiverPoint());
//     std::cout << p.firstBounceRange() << std::endl;
// }