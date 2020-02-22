#include "software/util/design_patterns/generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test generics with the factory here
class testGeneric
{
};

class testGenericA : public testGeneric
{
   public:
    static const std::string name;
};
const std::string testGenericA::name = "A";

class testGenericB : public testGeneric
{
   public:
    static const std::string name;
};
const std::string testGenericB::name = "B";

static TGenericFactory<std::string, testGeneric, testGenericA> testFactoryA;
static TGenericFactory<std::string, testGeneric, testGenericB> testFactoryB;


TEST(GenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((GenericFactory<std::string, testGeneric>::create("_FooBar_")),
                 std::invalid_argument);
}

TEST(GenericFactoryTest, test_create_generic_with_valid_name)
{
    auto type_ptr = GenericFactory<std::string, testGeneric>::create("A");

    EXPECT_TRUE(type_ptr);
}

TEST(GenericFactoryTest, test_get_registered_generic_names)
{
    auto registered_names =
        GenericFactory<std::string, testGeneric>::getRegisteredNames();
    EXPECT_EQ(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "A"), 1);
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "B"), 1);
}

TEST(GenericFactoryTest, test_get_registered_generic_constructors)
{
    auto registered_constructors =
        GenericFactory<std::string, testGeneric>::getRegisteredConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}
