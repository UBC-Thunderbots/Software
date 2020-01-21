#include "software/util/design_patterns/generic_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test generics with the factory here
class testGenericA {
};

class testGenericB : public testGenericA {
    public:
        static const std::string name;
};
const std::string testGenericB::name = "A";

static TGenericFactory<std::string, testGenericA, testGenericB> testFactory;

TEST(GenericFactoryTest, test_create_generic_with_invalid_name)
{
    EXPECT_THROW((GenericFactory<std::string, testGenericA>::createGeneric("_FooBar_")), std::invalid_argument);
}
//
//TEST(BackendFactoryTest, test_create_backend_with_valid_name)
//{
//    auto backend_ptr = BackendFactory::createBackend("A");
//
//    EXPECT_TRUE(backend_ptr);
//}
//
//TEST(BackendFactoryTest, test_get_registered_backend_names)
//{
//    auto registered_names = BackendFactory::getRegisteredBackendNames();
//    EXPECT_EQ(registered_names.size(), 2);
//    // Make sure we get the names we are expecting
//    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "A"), 1);
//    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "B"), 1);
//}
//
//TEST(BackendFactoryTest, test_get_registered_backend_constructors)
//{
//    auto registered_constructors = BackendFactory::getRegisteredBackendConstructors();
//    EXPECT_EQ(registered_constructors.size(), 2);
//}

// construct factory from valid and invalid string