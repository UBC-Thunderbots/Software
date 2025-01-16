#pragma once

#include <vector>

#include "software/logger/logger.h"

/**
 * The "sum-tree" (Fenwick tree) is a data structure that stores an array of values
 * and can efficiently compute the prefix sums of the values in O(log n) time.
 *
 * It is a binary tree where every node's value is equal to the sum of its children.
 * The leaf nodes store the array values and the internal nodes store intermediate sums,
 * with the root node storing the sum of all array values.
 *
 * Using a sum tree for a prioritized replay buffer to store transition priorities
 * allows updating of priorities and sampling from the replay buffer in O(log n) time.
 *
 * https://arxiv.org/pdf/1511.05952 (Appendix B.2.1, Proportional prioritization)
 *
 * @tparam TValue the type of value to store in the sum-tree
 * @tparam TData the type of data to store in the sum-tree
 */
template <typename TValue, typename TData>
class SumTree
{
    static_assert(std::is_arithmetic_v<TValue>, "TValue must be numeric");

   public:
    /**
     * Constructs a sum-tree with the given capacity.
     *
     * @param capacity the maximum capacity of the sum-tree
     */
    explicit SumTree(size_t capacity);

    /**
     * Sets the next available index in the sum-tree to the given value and
     * associates that index with the given data value. The sum-tree acts like
     * a circular buffer, so if the sum-tree is full, the oldest entry in the
     * sum-tree will be overwritten.
     *
     * Time complexity: O(log n)
     *
     * @param value the value to add to the sum-tree
     * @param data the data value to associate with the written-to sum-tree index
     * @return the index in the sum-tree where the value was written
     */
    size_t add(TValue value, TData data);

    /**
     * Sets the value at the given index in the sum-tree.
     *
     * Time complexity: O(log n)
     *
     * @param index the index of the value in the sum-tree to update
     * @param value the updated value
     */
    void update(size_t index, TValue value);

    /**
     * Gets the value at the given index in the sum-tree.
     *
     * Time complexity: O(1)
     *
     * @param index the index in the sum-tree
     * @return the value associated with the index
     */
    TValue getValue(size_t index) const;

    /**
     * Gets the data value associated with the given index in the sum-tree.
     *
     * Time complexity: O(1)
     *
     * @param index the index in the sum-tree
     * @return the data value associated with the index
     */
    TData getData(size_t index) const;

    /**
     * Returns the sum of all values in the sum-tree.
     *
     * Time complexity: O(1)
     *
     * @return the sum of all values in the sum-tree
     */
    TValue total() const;

    /**
     * Returns the number of values in the sum-tree.
     *
     * Time complexity: O(1)
     *
     * @return the number of values in the sum-tree
     */
    size_t size() const;

    /**
     * Finds the smallest index `i` in the sum-tree such that
     * sum(tree[0] + tree[1] + ... + tree[i]) <= prefix_sum.
     *
     * Time complexity: O(log n)
     *
     * @param prefix_sum the prefix sum to find
     * @return the smallest index in the sum-tree whose prefix sum is
     * less than or equal to the given prefix_sum
     */
    size_t findPrefixSumIndex(TValue prefix_sum) const;

   private:
    // The maximum capacity of the sum-tree
    size_t capacity_;

    // Number of elements in the sum-tree
    size_t size_;

    // Next index in the sum-tree to set
    size_t next_index_;

    // Stores the binary tree represented as an array
    // See https://en.wikipedia.org/wiki/Binary_tree#Arrays
    std::vector<TValue> nodes_;

    // Stores the data values associated with the sum-tree indices
    std::vector<TData> data_;
};

template <typename TValue, typename TData>
SumTree<TValue, TData>::SumTree(size_t capacity)
    : capacity_(capacity),
      size_(0),
      next_index_(0),
      nodes_(2 * capacity - 1),
      data_(capacity)
{
}

template <typename TValue, typename TData>
size_t SumTree<TValue, TData>::add(TValue value, TData data)
{
    const size_t index = next_index_;
    data_[index]       = data;
    update(index, value);

    next_index_ = (next_index_ + 1) % capacity_;
    size_       = std::min(size_ + 1, capacity_);

    return index;
}

template <typename TValue, typename TData>
void SumTree<TValue, TData>::update(size_t index, TValue value)
{
    size_t node_index         = index + capacity_ - 1;
    const TValue value_change = value - nodes_.at(node_index);

    while (node_index > 0)
    {
        nodes_[node_index] += value_change;
        node_index = (node_index - 1) / 2;
    }

    nodes_[0] += value_change;
}

template <typename TValue, typename TData>
TValue SumTree<TValue, TData>::getValue(size_t index) const
{
    return nodes_.at(index + capacity_ - 1);
}

template <typename TValue, typename TData>
TData SumTree<TValue, TData>::getData(size_t index) const
{
    return data_.at(index);
}

template <typename TValue, typename TData>
TValue SumTree<TValue, TData>::total() const
{
    return nodes_.front();
}

template <typename TValue, typename TData>
size_t SumTree<TValue, TData>::size() const
{
    return size_;
}

template <typename TValue, typename TData>
size_t SumTree<TValue, TData>::findPrefixSumIndex(TValue prefix_sum) const
{
    CHECK(prefix_sum <= total())
        << "Prefix sum must be less than or equal to the sum of all values in the sum-tree";

    size_t index = 0;
    while (2 * index + 1 < nodes_.size())
    {
        const size_t left  = 2 * index + 1;
        const size_t right = left + 1;
        if (prefix_sum <= nodes_.at(left))
        {
            index = left;
        }
        else
        {
            index = right;
            prefix_sum -= nodes_.at(left);
        }
    }

    return index - capacity_ + 1;
}
