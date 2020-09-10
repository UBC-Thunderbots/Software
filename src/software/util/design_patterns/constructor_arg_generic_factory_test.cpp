#include "software/util/design_patterns/constructor_arg_generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test generics with the factory here
class TestGeneric
{
   public:
    virtual std::string info() = 0;
};

class TestGenericStringArg : public TestGeneric
{
   public:
    TestGenericStringArg(std::string _text) : text(_text) {};
    std::string info() override {return text;}
    std::string text;
};

class TestGenericIntArg : public TestGeneric
{
   public:
    TestGenericIntArg(int _number) : number(_number) {};
    std::string info() override {return std::to_string(number);}
    int number;
};

class TestGenericTwoArgs : public TestGeneric
{
   public:
    TestGenericTwoArgs(int _number, std::string _string) : number(_number), string(_string) {
        std::cout << number << " " << string << std::endl;
    };
    std::string info() override {return std::to_string(number) + ' ' + string;}
    int number;
    std::string string;
};

static TConstructorArgGenericFactory<std::string, TestGeneric, TestGenericStringArg, std::string> testFactoryA;
static TConstructorArgGenericFactory<std::string, TestGeneric, TestGenericIntArg, int> testFactoryB;
static TMultipleConstructorArgGenericFactory<std::string, TestGeneric,
TestGenericTwoArgs(int, std::string)> testFactoryC;




TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((ConstructorArgGenericFactory<std::string, TestGeneric>::create("_FooBar_", 5)),
                 std::invalid_argument);
}

TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_invalid_arg_type)
{
    EXPECT_THROW((ConstructorArgGenericFactory<std::string, TestGeneric>::create(
        "TestGenericStringArg", std::any(5))),
                 std::invalid_argument);
}

TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_valid_name_and_arg_type)
{
    auto type_ptr = ConstructorArgGenericFactory<std::string, TestGeneric>::create(
        "TestGenericIntArg", 5);

    EXPECT_TRUE(type_ptr);
    EXPECT_EQ(static_cast<TestGenericIntArg*>(type_ptr.get())->number, 5);
}

TEST(ConstructorArgGenericFactoryTest, test_create_generic_with_two_args)
{
    auto type_ptr = ConstructorArgGenericFactory<std::string, TestGeneric>::create(
        "TestGenericTwoArgs", 5, std::string("five"));

    EXPECT_EQ(type_ptr->info(), "5 five");
}

TEST(ConstructorArgGenericFactoryTest, test_get_registered_generic_names)
{
    auto registered_names =
        ConstructorArgGenericFactory<std::string, TestGeneric>::getRegisteredNames();
    EXPECT_EQ(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "TestGenericStringArg"), 1);
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "TestGenericIntArg"), 1);
}

TEST(ConstructorArgGenericFactoryTest, test_get_registered_generic_constructors)
{
    auto registered_constructors =
        ConstructorArgGenericFactory<std::string, TestGeneric>::getRegisteredConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}
