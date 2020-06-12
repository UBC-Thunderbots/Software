#include "software/time/time.h"

#include <gtest/gtest.h>

TEST(TimeTest, test_now)
{
    double t1 = Time::now();
    double t2 = Time::now();
    EXPECT_NE(t1, t2);
}
