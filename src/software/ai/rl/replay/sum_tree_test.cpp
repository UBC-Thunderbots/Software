#include "software/ai/rl/replay/sum_tree.hpp"

#include <gtest/gtest.h>

class SumTreeTest : public ::testing::Test
{
   protected:
    SumTree<int, int> tree;
    SumTreeTest() : tree(4) {}
};

TEST_F(SumTreeTest, add_and_get_value)
{
    EXPECT_EQ(tree.size(), 0);
    EXPECT_EQ(tree.total(), 0);

    size_t index_0 = tree.add(10, 100);
    size_t index_1 = tree.add(20, 200);
    size_t index_2 = tree.add(30, 300);

    EXPECT_EQ(tree.size(), 3);
    EXPECT_EQ(tree.getValue(index_0), 10);
    EXPECT_EQ(tree.getValue(index_1), 20);
    EXPECT_EQ(tree.getValue(index_2), 30);
    EXPECT_EQ(tree.getData(index_0), 100);
    EXPECT_EQ(tree.getData(index_1), 200);
    EXPECT_EQ(tree.getData(index_2), 300);
    EXPECT_EQ(tree.total(), 10 + 20 + 30);
}

TEST_F(SumTreeTest, update_value)
{
    size_t index_0 = tree.add(10, 100);
    size_t index_1 = tree.add(20, 200);

    EXPECT_EQ(tree.getValue(index_0), 10);
    EXPECT_EQ(tree.getValue(index_1), 20);
    EXPECT_EQ(tree.total(), 30);

    tree.update(index_0, 15);

    EXPECT_EQ(tree.getValue(index_0), 15);
    EXPECT_EQ(tree.getValue(index_1), 20);
    EXPECT_EQ(tree.total(), 35);
}

TEST_F(SumTreeTest, add_when_buffer_full)
{
    tree.add(10, 100);
    tree.add(20, 200);
    tree.add(30, 300);
    tree.add(40, 400);

    EXPECT_EQ(tree.total(), 10 + 20 + 30 + 40);
    EXPECT_EQ(tree.getValue(0), 10);

    // Adding value to full buffer should replace oldest entry at index 0
    // (circular buffer behaviour)
    size_t index = tree.add(50, 500);
    EXPECT_EQ(tree.size(), 4);
    EXPECT_EQ(index, 0);
    EXPECT_EQ(tree.getValue(0), 50);
    EXPECT_EQ(tree.total(), 20 + 30 + 40 + 50);
}

TEST_F(SumTreeTest, find_prefix_sum_index)
{
    tree.add(10, 100);
    tree.add(20, 200);
    tree.add(30, 300);

    EXPECT_EQ(tree.findPrefixSumIndex(0), 0);
    EXPECT_EQ(tree.findPrefixSumIndex(10), 0);
    EXPECT_EQ(tree.findPrefixSumIndex(11), 1);
    EXPECT_EQ(tree.findPrefixSumIndex(20), 1);
    EXPECT_EQ(tree.findPrefixSumIndex(30), 1);
    EXPECT_EQ(tree.findPrefixSumIndex(31), 2);
    EXPECT_EQ(tree.findPrefixSumIndex(40), 2);
    EXPECT_EQ(tree.findPrefixSumIndex(50), 2);
    EXPECT_EQ(tree.findPrefixSumIndex(60), 2);
}
