#include "ai/hl/stp/play/play_factory.h"

#include <gtest/gtest.h>

#include <exception>

#include "test/ai/hl/stp/test_plays/move_test_play.h"
#include "test/ai/hl/stp/test_plays/stop_test_play.h"
#include "test/test_util/test_util.h"

TEST(PlayFactoryTest, test_create_play_with_invalid_name)
{
    EXPECT_THROW(PlayFactory::createPlay("_FooBar_"), std::invalid_argument);
}

TEST(PlayFactoryTest, test_create_play_with_valid_name)
{
    auto play_ptr = PlayFactory::createPlay(MoveTestPlay::name);

    EXPECT_TRUE(play_ptr);
}

TEST(PlayFactoryTest, test_get_registered_play_names)
{
    auto registered_names = PlayFactory::getRegisteredPlayNames();
    // Make sure we get the right number of names. There should be at least as many
    // as the number of TestPlays, since the number of "real" Plays can change a lot
    EXPECT_GE(registered_names.size(), 2);
    // Make sure we get the names we are expecting
    EXPECT_EQ(
        std::count(registered_names.begin(), registered_names.end(), MoveTestPlay::name),
        1);
    EXPECT_EQ(
        std::count(registered_names.begin(), registered_names.end(), StopTestPlay::name),
        1);
}

TEST(PlayFactoryTest, test_get_registered_play_constructors)
{
    auto registered_constructors = PlayFactory::getRegisteredPlayConstructors();
    // Make sure we get the right number of constructor. There should be at least as many
    // as the number of TestPlays, since the number of "real" Plays can change a lot
    EXPECT_GE(registered_constructors.size(), 2);
}
