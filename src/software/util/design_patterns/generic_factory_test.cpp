#include "software/util/design_patterns/generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test generics with the factory here
class TestGeneric
{
};

class TestGenericA : public TestGeneric
{
};

class TestGenericB : public TestGeneric
{
};

static TGenericFactory<std::string, TestGeneric, TestGenericA> testFactoryA;
static TGenericFactory<std::string, TestGeneric, TestGenericB> testFactoryB;


TEST(GenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((GenericFactory<std::string, TestGeneric>::create("_FooBar_")),
                 std::invalid_argument);
}

TEST(GenericFactoryTest, test_create_generic_with_valid_name)
{
    auto type_ptr = GenericFactory<std::string, TestGeneric>::create("TestGenericA");

    EXPECT_TRUE(type_ptr);
}

TEST(GenericFactoryTest, test_get_registered_generic_names)
{
    auto registered_names =
        GenericFactory<std::string, TestGeneric>::getRegisteredNames();
    EXPECT_EQ(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(
        std::count(registered_names.begin(), registered_names.end(), "TestGenericA"), 1);
    EXPECT_EQ(
        std::count(registered_names.begin(), registered_names.end(), "TestGenericB"), 1);
}

TEST(GenericFactoryTest, test_get_registered_generic_constructors)
{
    auto registered_constructors =
        GenericFactory<std::string, TestGeneric>::getRegisteredConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}
