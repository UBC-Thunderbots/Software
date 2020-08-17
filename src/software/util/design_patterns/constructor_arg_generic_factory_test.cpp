#include "software/util/design_patterns/constructor_arg_generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test generics with the factory here
class testGeneric
{

};

class testGenericStringArg : public testGeneric
{
   public:
    testGenericStringArg(std::string _text) : text(_text) {};
    static const std::string name;
    std::string text;
};
const std::string testGenericStringArg::name = "testGenericStringArg";

class testGenericIntArg : public testGeneric
{
   public:
    testGenericIntArg(int _number) : number(_number) {};
    static const std::string name;
    int number;
};
const std::string testGenericIntArg::name = "testGenericIntArg";

static TConstructorArgGenericFactory<std::string, testGeneric, testGenericStringArg, std::string> testFactoryA;
static TConstructorArgGenericFactory<std::string, testGeneric, testGenericIntArg, int> testFactoryB;


TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((ConstructorArgGenericFactory<std::string, testGeneric>::create("_FooBar_", 5)),
                 std::invalid_argument);
}

TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_invalid_arg_type)
{
    EXPECT_THROW((ConstructorArgGenericFactory<std::string, testGeneric>::create(
        "testGenericStringArg", std::any(5))),
                 std::invalid_argument);
}

TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_valid_name_and_arg_type)
{
    auto type_ptr = ConstructorArgGenericFactory<std::string, testGeneric>::create(
        "testGenericIntArg", 5);

    EXPECT_TRUE(type_ptr);
    EXPECT_EQ(static_cast<testGenericIntArg*>(type_ptr.get())->number, 5);
}

TEST(ConstructorArgGenericFactoryTest, test_get_registered_generic_names)
{
    auto registered_names =
        ConstructorArgGenericFactory<std::string, testGeneric>::getRegisteredNames();
    EXPECT_EQ(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "testGenericStringArg"), 1);
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "testGenericIntArg"), 1);
}

TEST(ConstructorArgGenericFactoryTest, test_get_registered_generic_constructors)
{
    auto registered_constructors =
        ConstructorArgGenericFactory<std::string, testGeneric>::getRegisteredConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}
