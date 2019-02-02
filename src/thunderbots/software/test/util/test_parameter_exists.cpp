#include <gtest/gtest.h>
#include <ros/ros.h>

#include "util/parameter/dynamic_parameters.h"
#include "util/parameter/parameter.h"

namespace
{
    std::vector<Parameter<int32_t>> int_parameters;
    std::vector<Parameter<double>> double_parameters;
    std::vector<Parameter<bool>> bool_parameters;
    std::vector<Parameter<std::string>> string_parameters;
}  // namespace

class CheckParameterExistanceInt32 : public ::testing::TestWithParam<Parameter<int32_t>>
{
};

TEST_P(CheckParameterExistanceInt32, Int32Test)
{ 
    auto val = GetParam();
}

class CheckParameterExistanceDouble : public ::testing::TestWithParam<Parameter<double>>
{
};

TEST_P(CheckParameterExistanceDouble, DoubleTest)
{
    auto val = GetParam();
}

class CheckParameterExistanceBool : public ::testing::TestWithParam<Parameter<bool>>
{
};

TEST_P(CheckParameterExistanceBool, BoolTest)
{
    auto val = GetParam();
}

class CheckParameterExistanceStr : public ::testing::TestWithParam<Parameter<std::string>>
{
};

TEST_P(CheckParameterExistanceStr, StringTest)
{
    auto val = GetParam();
}

INSTANTIATE_TEST_CASE_P(Int32Test, CheckParameterExistanceInt32,
        ::testing::ValuesIn(int_parameters.begin(), int_parameters.end()));
INSTANTIATE_TEST_CASE_P(DoubleTest, CheckParameterExistanceDouble,
        ::testing::ValuesIn(double_parameters.begin(), double_parameters.end()));
INSTANTIATE_TEST_CASE_P(BoolTest, CheckParameterExistanceBool,
        ::testing::ValuesIn(bool_parameters.begin(), bool_parameters.end()));
INSTANTIATE_TEST_CASE_P(StringTest, CheckParameterExistanceStr,
        ::testing::ValuesIn(string_parameters.begin(), string_parameters.end()));

int main(int argc, char** argv)
{
    for (const auto& pair : Parameter<int32_t>::getRegistry())
    {
        int_parameters.push_back(*pair.second);
    }
    for (const auto& pair : Parameter<double>::getRegistry())
    {
        double_parameters.push_back(*pair.second);
    }
    for (const auto& pair : Parameter<bool>::getRegistry())
    {
        bool_parameters.push_back(*pair.second);
    }
    for (const auto& pair : Parameter<std::string>::getRegistry())
    {
        string_parameters.push_back(*pair.second);
    }
    // populate global parameter vectors
    ros::init(argc, argv, "parameter_existance_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
