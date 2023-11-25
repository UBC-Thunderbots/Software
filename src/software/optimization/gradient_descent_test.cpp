#include <gtest/gtest.h>

#include <cmath>

#include "software/optimization/gradient_descent_optimizer.hpp"

TEST(GradientDescentOptimizerTest, minimize_single_valued_function)
{
    GradientDescentOptimizer<1> gradientDescentOptimizer({0.1});

    // f = x^2
    auto f = [](std::array<double, 1> x) { return std::pow(x.at(0), 2.0); };

    auto min = gradientDescentOptimizer.minimize(f, {1}, 100);

    EXPECT_NEAR(min.at(0), 0, 0.1);
}

TEST(GradientDescentOptimizerTest, maximize_single_valued_function)
{
    GradientDescentOptimizer<1> gradientDescentOptimizer({0.1});

    // f = -x^2
    auto f = [](std::array<double, 1> x) { return -std::pow(x.at(0), 2.0); };

    auto max = gradientDescentOptimizer.maximize(f, {1}, 100);

    EXPECT_NEAR(max.at(0), 0, 0.1);
}

TEST(GradientDescentOptimizerTest, minimize_multi_valued_function)
{
    // Note that we halve the weight for "y" here to make sure
    // gradient descent can see the function in a more homogeneous way.
    // See the GradientDescentOptimizer class javadoc comment for more details.
    GradientDescentOptimizer<2> gradientDescentOptimizer({0.1, 0.05});

    // f = x^2 + 2*y^2 + 20
    auto f = [](std::array<double, 2> x) {
        return std::pow(x.at(0), 2) + 2 * std::pow(x.at(1), 2) + 20;
    };

    auto min = gradientDescentOptimizer.minimize(f, {1, -1}, 100);

    EXPECT_NEAR(min.at(0), 0, 0.1);
    EXPECT_NEAR(min.at(1), 0, 0.1);
}

TEST(GradientDescentOptimizerTest, minimize_multi_valued_function_with_offsets)
{
    // Note that we halve the weight for "y" here to make sure
    // gradient descent can see the function in a more homogeneous way.
    // See the GradientDescentOptimizer class javadoc comment for more details.
    GradientDescentOptimizer<2> gradientDescentOptimizer({0.1, 0.05});

    // f = (x+5)^2 + 2*(y-4)^2 + 20
    auto f = [](std::array<double, 2> x) {
        return std::pow(x.at(0) + 5, 2) + 2 * std::pow(x.at(1) - 4, 2) + 20;
    };

    auto min = gradientDescentOptimizer.minimize(f, {0, 0}, 150);

    EXPECT_NEAR(min.at(0), -5, 0.1);
    EXPECT_NEAR(min.at(1), 4, 0.1);
}

TEST(GradientDescentOptimizerTest, maximize_sigmoid)
{
    GradientDescentOptimizer<1> gradientDescentOptimizer({0.1});

    // f = 1 / (1 + exp(2-2x))
    auto f = [](std::array<double, 1> x) { return 1 / (1 + std::exp(2 - 2 * x[0])); };

    auto min = gradientDescentOptimizer.maximize(f, {0}, 100);

    // We expect that the gradient descent will make it over the
    // main part of the "S" in the sigmoid
    EXPECT_GE(min.at(0), 3);
}

TEST(GradientDescentOptimizer, maximize_sigmoid_performance_test)
{
    // This test can be used to judge if the performance of gradient descent
    // has decreased, as we here we are checking if it took _exactly_ the number
    // of iterations to get past the main "S" part of a sigmoid

    GradientDescentOptimizer<1> gradientDescentOptimizer({0.1});

    // f = 1 / (1 + exp(2-2x))
    auto f = [](std::array<double, 1> x) { return 1 / (1 + std::exp(2 - 2 * x[0])); };

    const unsigned int EXACT_NUMBER_OF_ITERATIONS_TO_PASS_S_CURVE = 23;

    auto min = gradientDescentOptimizer.maximize(
        f, {0}, EXACT_NUMBER_OF_ITERATIONS_TO_PASS_S_CURVE);

    // We expect that the gradient descent will make it over the main part of
    // the "S" in the sigmoid within the given number of iterations
    EXPECT_GE(min.at(0), 3);
}
