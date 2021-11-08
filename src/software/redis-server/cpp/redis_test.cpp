#include <gtest/gtest.h>
#include <sw/redis++/redis++.h>
#include <iostream>
#include <string>


TEST(ServerTest, Get)
{
    auto redis = sw::redis::Redis("tcp://127.0.0.1:6379");
    // redis.set("1","2");
    EXPECT_EQ(3, 3);
}