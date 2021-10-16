#include "software/util/generic_factory/generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

#include "software/util/generic_factory/generic_factory.h"

class TestConfig
{
};

// Create and register two test generics with the factory here
class TestGeneric
{
};

class TestGenericA : public TestGeneric
{
   public:
    TestGenericA(std::shared_ptr<const TestConfig> config);
};

TestGenericA::TestGenericA(std::shared_ptr<const TestConfig> config) {}

class TestGenericB : public TestGeneric
{
   public:
    TestGenericB(std::shared_ptr<const TestConfig> config);
};

TestGenericB::TestGenericB(std::shared_ptr<const TestConfig> config) {}

static TGenericFactory<std::string, TestGeneric, TestGenericA, TestConfig> testFactoryA;
static TGenericFactory<std::string, TestGeneric, TestGenericB, TestConfig> testFactoryB;


TEST(GenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((GenericFactory<std::string, TestGeneric, TestConfig>::create(
                     "_FooBar_", std::make_shared<const TestConfig>())),
                 std::invalid_argument);
}

TEST(GenericFactoryTest, test_create_generic_with_valid_name)
{
    auto type_ptr = GenericFactory<std::string, TestGeneric, TestConfig>::create(
        "TestGenericA", std::make_shared<const TestConfig>());

    EXPECT_TRUE(type_ptr);
}

TEST(GenericFactoryTest, test_get_registered_generic_names)
{
    auto registered_names =
        GenericFactory<std::string, TestGeneric, TestConfig>::getRegisteredNames();
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
        GenericFactory<std::string, TestGeneric, TestConfig>::getRegisteredConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}
