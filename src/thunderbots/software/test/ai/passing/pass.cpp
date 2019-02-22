/**
 * Tests for the "Pass" class
 */

#include "ai/passing/pass.h"

#include <gtest/gtest.h>

using namespace AI::Passing;

TEST(PassTest, test_getters)
{
    Pass p(Point(1, 2), Point(3, 4), 99.97, Timestamp::fromSeconds(10));

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    EXPECT_EQ(99.97, p.speed());
    EXPECT_EQ(Timestamp::fromSeconds(10), p.startTime());
}

TEST(PassTest, test_stream_operator)
{
    Pass p(Point(1, 2), Point(3, 4), 99.97, Timestamp::fromSeconds(10));

    std::stringstream out;
    out << p;
    EXPECT_EQ("Receiver: (3, 4), Passer: (1, 2) Speed (m/s): 99.97 Start Time (s): 10",
              out.str());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
