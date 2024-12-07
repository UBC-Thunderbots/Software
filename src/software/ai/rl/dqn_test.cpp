#include "dqn.hpp"

#include <gtest/gtest.h>

struct TestState
{
    torch::Tensor toTensor()
    {
        return torch::ones(size());
    }

    static constexpr size_t size()
    {
        return 5;
    }
};

MAKE_ENUM(TestAction, action1, action2, action3)

TEST(DQNTest, dqn_test)
{
    DQN<TestState, TestAction> dqn(0.3f, 0.1f, 0.3f);
}
