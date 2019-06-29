#include <gtest/gtest.h>

#include "ai/util.h"

TEST(comma_separated_list_to_set_1, CommaSeparatedListToSetTest)
{
    std::string input("1, 2, 3, 4, 5, 6, 7, 8");
    std::unordered_set<unsigned int> output = commaSeparatedListToSet(input);
    std::unordered_set<unsigned int> expected{1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT_EQ(output, expected);
}

TEST(comma_separated_list_to_set_2, CommaSeparatedListToSetTest)
{
    std::string input("1");
    std::unordered_set<unsigned int> output = commaSeparatedListToSet(input);
    std::unordered_set<unsigned int> expected{1};
    EXPECT_EQ(output, expected);
}

TEST(comma_separated_list_to_set_3, CommaSeparatedListToSetTest)
{
    std::string input("1,2,");
    std::unordered_set<unsigned int> output = commaSeparatedListToSet(input);
    std::unordered_set<unsigned int> expected{1, 2};
    EXPECT_EQ(output, expected);
}

TEST(comma_separated_list_to_set_exception, CommaSeparatedListToSetTest)
{
    std::string input("1, wadwadwadwad, 3");
    std::unordered_set<unsigned int> output = commaSeparatedListToSet(input);
    // we should be robust against non-number/comma symbols
    std::unordered_set<unsigned int> expected{1, 3};
    EXPECT_EQ(output, expected);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}