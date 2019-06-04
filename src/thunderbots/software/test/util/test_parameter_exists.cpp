#include <gtest/gtest.h>
#include <ros/ros.h>

#include <chrono>
#include <thread>

#include "util/parameter/dynamic_parameters.h"
#include "util/parameter/parameter.h"

namespace
{
    // vectors to store the parameters fetched from the registry
    std::vector<Parameter<int32_t>> int_parameters;
    std::vector<Parameter<double>> double_parameters;
    std::vector<Parameter<bool>> bool_parameters;
    std::vector<Parameter<std::string>> string_parameters;

    // how long to wait for before running the tests, allowing time for the node to spawn
    const int WAIT_FOR_PARAMS_SECONDS = 5;
    // msg to display before the name of the paramter that is not found
    const std::string MISMATCH_MSG = "Parameter mismatch with: ";
}  // namespace

class CheckParameterExistanceInt32 : public ::testing::TestWithParam<Parameter<int32_t>>
{
    // fixture used to run paramterized int32_t existance test
};

TEST_P(CheckParameterExistanceInt32, Int32Test)
{
    ASSERT_TRUE(GetParam().existsInParameterServer())
        << MISMATCH_MSG << GetParam().getROSParameterPath();
}

class CheckParameterExistanceDouble : public ::testing::TestWithParam<Parameter<double>>
{
    // fixture used to run paramterized double existance test
};

TEST_P(CheckParameterExistanceDouble, DoubleTest)
{
    ASSERT_TRUE(GetParam().existsInParameterServer())
        << MISMATCH_MSG << GetParam().getROSParameterPath();
}

class CheckParameterExistanceBool : public ::testing::TestWithParam<Parameter<bool>>
{
    // fixture used to run paramterized bool existance test
};

TEST_P(CheckParameterExistanceBool, BoolTest)
{
    ASSERT_TRUE(GetParam().existsInParameterServer())
        << MISMATCH_MSG << GetParam().getROSParameterPath();
}

class CheckParameterExistanceStr : public ::testing::TestWithParam<Parameter<std::string>>
{
    // fixture used to run paramterized string existance test
};

TEST_P(CheckParameterExistanceStr, StringTest)
{
    ASSERT_TRUE(GetParam().existsInParameterServer())
        << MISMATCH_MSG << GetParam().getROSParameterPath();
}

// setup all test cases for the 4 XmlRpc types
INSTANTIATE_TEST_CASE_P(Int32Test, CheckParameterExistanceInt32,
                        ::testing::ValuesIn(int_parameters.begin(),
                                            int_parameters.end()));
INSTANTIATE_TEST_CASE_P(DoubleTest, CheckParameterExistanceDouble,
                        ::testing::ValuesIn(double_parameters.begin(),
                                            double_parameters.end()));
INSTANTIATE_TEST_CASE_P(BoolTest, CheckParameterExistanceBool,
                        ::testing::ValuesIn(bool_parameters.begin(),
                                            bool_parameters.end()));
INSTANTIATE_TEST_CASE_P(StringTest, CheckParameterExistanceStr,
                        ::testing::ValuesIn(string_parameters.begin(),
                                            string_parameters.end()));

int main(int argc, char** argv)
{
    // grab parameters of each type from its respective registry
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

    // run tests
    ros::init(argc, argv, "parameter_existance_test");
    testing::InitGoogleTest(&argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(WAIT_FOR_PARAMS_SECONDS));
    return RUN_ALL_TESTS();
}
