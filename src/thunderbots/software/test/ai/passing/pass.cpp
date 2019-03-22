/**
 * Tests for the "Pass" class
 */

#include "ai/passing/pass.h"

#include <gtest/gtest.h>

using namespace AI::Passing;

TEST(PassTest, test_constructing_pass_with_negative_speed)
{
    EXPECT_THROW(Pass(Point(1, 2), Point(3, 4), -0.1, Timestamp::fromSeconds(10)),
                 std::invalid_argument);
}

TEST(PassTest, test_getters)
{
    Pass p(Point(1, 2), Point(3, 4), 3.443, Timestamp::fromSeconds(10));

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    EXPECT_EQ(3.443, p.speed());
    EXPECT_DOUBLE_EQ(std::sqrt(8) / 3.443 + 10, p.estimateReceiveTime().getSeconds());
    EXPECT_DOUBLE_EQ(std::sqrt(8) / 3.443, p.estimatePassDuration().getSeconds());
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
