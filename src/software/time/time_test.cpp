#include "software/time/time.h"

#include <gtest/gtest.h>

TEST(TimeTest, test_now)
{
    double t1 = Time::now();
    // Add some time in between
    EXPECT_EQ(t1, t1);
    double t2 = Time::now();
    EXPECT_TRUE(t2 > t1);
}
