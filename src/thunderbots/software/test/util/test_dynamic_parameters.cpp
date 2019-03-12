#include <gtest/gtest.h>
#include <ros/ros.h>

#include <random>

#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/parameter.h"

/* * Registry Test Fixutre:
 *      This fixture is used everywhere to return a unique_value value
 *      specific to the type its instantiated with.
 */
template <typename T>
class RegistryTest : public ::testing::Test
{
   protected:
    RegistryTest()
    {
        value_ = get_unique_value();
    }

    /*
     * Returns a unique value, according to the
     * constructed type
     *
     * NOTE: The tests return a changing value every time to
     * help test when the parameter value changes
     *
     * The boolean alternates between True/False
     */
    T get_unique_value()
    {
        if constexpr (std::is_same<T, bool>::value)
        {
            return (unique_value_counter += 1) & 1;
        }
        else if constexpr (std::is_same<T, int32_t>::value)
        {
            return unique_value_counter += 1;
        }
        else if constexpr (std::is_same<T, double>::value)
        {
            return static_cast<double>(unique_value_counter += 1);
        }
        else if constexpr (std::is_same<T, std::string>::value)
        {
            return std::to_string(unique_value_counter += 1);
        }
        else
        {
            ADD_FAILURE() << "Attempting to test with unsupported type";
        }
    }

    /*
     * Returns a unique string to use for
     * a parameter name (unique enough for our tests)
     */
    std::string get_unique_param_name()
    {
        return std::to_string(std::rand());
    }

    int unique_value_counter = 0;
    T value_;
};

/**********************************************************************
 *                               TESTS                                *
 **********************************************************************/
// all tests are grouped into the RegistryTest test case

// gtest getROSParameterPath
TEST(RegistryTest, get_param_test)
{
    Parameter<bool> test_param = Parameter<bool>("test_param", "parameters", false);
    EXPECT_EQ(test_param.getROSParameterPath(), "/parameters/test_param");
}

// typed test to test the constructor for every supported XmlRpc Type
using XmlRpcTypes = ::testing::Types<bool, int32_t, std::string, double>;
TYPED_TEST_CASE(RegistryTest, XmlRpcTypes);

// test to make sure the call to registerParameter works as expected
TYPED_TEST(RegistryTest, constructor_test)
{
    // each type gets its own registry, same name should not conflict
    auto unique_value = RegistryTest<TypeParam>::get_unique_value();
    Parameter<TypeParam> test_param =
        Parameter<TypeParam>("test_all_type_params", "parameters", unique_value);
    try
    {
        ASSERT_EQ(test_param.getRegistry().count("test_all_type_params"), 1);
        ASSERT_EQ(test_param.getRegistry().at("test_all_type_params")->value(),
                  unique_value);
    }
    catch (...)
    {
        FAIL() << "Uncaught exception occured";
    }
}

// rostest UpdateParameterFromRosParameterServer (for all XmlRpcTypes)
TYPED_TEST(RegistryTest, update_parameter_test)
{
    ros::NodeHandle nh_;
    TypeParam unique_value = RegistryTest<TypeParam>::get_unique_value();

    // set ros param
    std::string unique_param_name = RegistryTest<TypeParam>::get_unique_param_name();
    nh_.setParam("/test/" + unique_param_name, unique_value);

    // create parameter
    Parameter<TypeParam> test_param =
        Parameter<TypeParam>(unique_param_name, "test", unique_value);

    // sanity check
    TypeParam stored_val;
    nh_.getParam("/test/" + unique_param_name, stored_val);
    ASSERT_EQ(test_param.value(), stored_val);

    // change param (this would normally be through the reconf GUI)
    unique_value = RegistryTest<TypeParam>::get_unique_value();
    nh_.setParam("/test/" + unique_param_name, unique_value);
    nh_.getParam("/test/" + unique_param_name, stored_val);
    ASSERT_NE(test_param.value(), stored_val);

    // Calling updateAll...() will loop through the registry and update all
    // the values (internally calling updateValueFromROSParameterServer on each param in
    // the registry) this is due to the different references after calling make_shared
    Util::DynamicParameters::updateAllParametersFromROSParameterServer();
    ASSERT_EQ(test_param.value(), stored_val);
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_parameters_ros_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
