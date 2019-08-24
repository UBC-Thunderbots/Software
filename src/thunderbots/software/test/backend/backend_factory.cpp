#include "backend/backend_factory.h"

#include <gtest/gtest.h>

#include <exception>
#include <iostream>

// Create and register two test backends with the factory here
class BackendA : public Backend
{
   public:
    static const std::string name;
};
const std::string BackendA::name = "A";
static TBackendFactory<BackendA> factoryA;
class BackendB : public Backend
{
   public:
    static const std::string name;
};
const std::string BackendB::name = "B";
// Register this backend in the BackendFactory
static TBackendFactory<BackendB> factoryB;


TEST(BackendFactoryTest, test_create_backend_with_invalid_name)
{
    EXPECT_THROW(BackendFactory::createBackend("_FooBar_"), std::invalid_argument);
}

TEST(BackendFactoryTest, test_create_backend_with_valid_name)
{
    auto backend_ptr = BackendFactory::createBackend("A");

    EXPECT_TRUE(backend_ptr);
}

TEST(BackendFactoryTest, test_get_registered_backend_names)
{
    auto registered_names = BackendFactory::getRegisteredBackendNames();
    EXPECT_EQ(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "A"), 1);
    EXPECT_EQ(std::count(registered_names.begin(), registered_names.end(), "B"), 1);
}

TEST(BackendFactoryTest, test_get_registered_backend_constructors)
{
    auto registered_constructors = BackendFactory::getRegisteredBackendConstructors();
    EXPECT_EQ(registered_constructors.size(), 2);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
