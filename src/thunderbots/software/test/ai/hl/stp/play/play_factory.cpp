#include "ai/hl/stp/play/play_factory.h"

#include <gtest/gtest.h>

#include <exception>

#include "test/test_util/test_util.h"

TEST(PlayFactoryTest, test_create_play_with_invalid_name)
{
    EXPECT_THROW(PlayFactory::createPlay("_FooBar_"), std::invalid_argument);
}

TEST(PlayFactoryTest, test_create_play_with_valid_name)
{
    auto play_ptr = PlayFactory::createPlay("Example Play");

    EXPECT_TRUE(play_ptr);
}

TEST(PlayFactoryTest, test_get_registered_play_names)
{
    auto registered_names = PlayFactory::getRegisteredPlayNames();
    // Since the number of Plays will constantly be changing, we make sure this function
    // works by checking that it returns at least 1 Play name, indicating registered plays
    // do show up the in the registered names list
    EXPECT_GE(registered_names.size(), 1);
}
