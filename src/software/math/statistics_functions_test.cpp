#include "software/math/statistics_functions.hpp"

#include <gtest/gtest.h>

TEST(StatisticsFunctionsTest, testMeanNoData)
{
    EXPECT_DOUBLE_EQ(mean(std::vector<int>()), 0.0);
}

TEST(StatisticsFunctionsTest, testMeanOneData)
{
    EXPECT_DOUBLE_EQ(mean(std::vector<int>{1}), 1.0);
}

TEST(StatisticsFunctionsTest, testMeanRepeatedData)
{
    EXPECT_DOUBLE_EQ(mean(std::vector<int>{1, 1, 1, 1, 1}), 1.0);
}

TEST(StatisticsFunctionsTest, testMeanDifferentDataNoRounding)
{
    EXPECT_DOUBLE_EQ(mean(std::vector<int>{1, 2, 3, 4, 5}), 3.0);
}

TEST(StatisticsFunctionsTest, testMeanDifferentDataWithRounding)
{
    EXPECT_DOUBLE_EQ(mean(std::vector<int>{1, 2, 3, 4, 5, 6}), 3.5);
}

TEST(StatisticsFunctionsTest, testStdevSampleNoData)
{
    EXPECT_DOUBLE_EQ(stdevSample(std::vector<int>()), 0.0);
}

TEST(StatisticsFunctionsTest, testStdevSampleOneData)
{
    EXPECT_DOUBLE_EQ(stdevSample(std::vector<int>{1}), 0.0);
}

TEST(StatisticsFunctionsTest, testStdevSampleTwoRepeatedData)
{
    EXPECT_DOUBLE_EQ(stdevSample(std::vector<int>{2, 2}), 0.0);
}

TEST(StatisticsFunctionsTest, testStdevSampleDifferentData)
{
    EXPECT_DOUBLE_EQ(stdevSample(std::vector<int>{1, 2, 3, 4, 5}), std::sqrt(10 / 4.0));
}
