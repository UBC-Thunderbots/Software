#include "software/util/typename/typename.h"

#include <gtest/gtest.h>

#include <memory>

class TestType
{
    virtual void foo() = 0;
};

class TestTypeA : public TestType
{
    void foo() override {}
};

TEST(TypeNameTest, abstract_base_class_concrete_subtype)
{
    std::shared_ptr<TestType> test_type = std::make_shared<TestTypeA>();
#ifdef __APPLE__
    // With clang / libc++, the standard library uses an inline namespace '__1' for
    // ABI versioning, which is reflected in the demangled type name.
    EXPECT_EQ("std::__1::shared_ptr<TestType>", objectTypeName(test_type));
#else
    EXPECT_EQ("std::shared_ptr<TestType>", objectTypeName(test_type));
#endif
    EXPECT_EQ("TestTypeA", objectTypeName(*test_type));
}

TEST(TypeNameTest, test_class_type_name)
{
    EXPECT_EQ("TestType", TYPENAME(TestType));
    EXPECT_EQ("TestTypeA", TYPENAME(TestTypeA));
}
