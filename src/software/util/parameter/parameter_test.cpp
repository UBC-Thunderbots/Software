#include "software/util/parameter/parameter.h"

#include <gtest/gtest.h>

#include <optional>

TEST(ParameterTest, register_single_callback_test)
{
    Parameter<bool> test_param =
        Parameter<bool>("test_param", false, {}, std::nullopt, std::nullopt);
    bool test_value = false;

    // This callback will set the test_value (by reference) to the given value
    auto callback = [&test_value](bool new_value) { test_value = new_value; };
    test_param.registerCallbackFunction(callback);

    EXPECT_FALSE(test_value);
    test_param.setValue(true);
    EXPECT_TRUE(test_value);
}

TEST(ParameterTest, register_multiple_unique_callbacks_test)
{
    Parameter<double> test_param =
        Parameter<double>("test_param", 0.0, {}, std::nullopt, std::nullopt);
    double test_value_1 = 1.0;
    double test_value_2 = -3.0;
    double test_value_3 = 4.5;

    auto callback_1 = [&test_value_1](double new_value) { test_value_1 += new_value; };
    test_param.registerCallbackFunction(callback_1);

    auto callback_2 = [&test_value_2](double new_value) { test_value_2 -= new_value; };
    test_param.registerCallbackFunction(callback_2);

    auto callback_3 = [&test_value_3](double new_value) { test_value_3 *= new_value; };
    test_param.registerCallbackFunction(callback_3);

    EXPECT_DOUBLE_EQ(test_value_1, 1.0);
    EXPECT_DOUBLE_EQ(test_value_2, -3.0);
    EXPECT_DOUBLE_EQ(test_value_3, 4.5);
    test_param.setValue(2.0);
    EXPECT_DOUBLE_EQ(test_value_1, 3.0);
    EXPECT_DOUBLE_EQ(test_value_2, -5.0);
    EXPECT_DOUBLE_EQ(test_value_3, 9);
}

TEST(ParameterTest, register_duplicate_callbacks_test)
{
    Parameter<int> test_param =
        Parameter<int>("test_param", 0, {}, std::nullopt, std::nullopt);
    int test_value = 0;

    // This callback will set the test_value (by reference) to the given value
    auto callback = [&test_value](int new_value) { test_value += new_value; };
    test_param.registerCallbackFunction(callback);
    test_param.registerCallbackFunction(callback);

    EXPECT_EQ(test_value, 0);
    test_param.setValue(1);
    EXPECT_EQ(test_value, 2);
}
