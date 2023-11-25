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
    EXPECT_EQ("std::shared_ptr<TestType>", objectTypeName(test_type));
    EXPECT_EQ("TestTypeA", objectTypeName(*test_type));
}

TEST(TypeNameTest, test_class_type_name)
{
    EXPECT_EQ("TestType", TYPENAME(TestType));
    EXPECT_EQ("TestTypeA", TYPENAME(TestTypeA));
}
