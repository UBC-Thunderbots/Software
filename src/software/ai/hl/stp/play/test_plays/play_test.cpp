#include "software/ai/hl/stp/play/test_plays/move_test_play.h"

#include <gtest/gtest.h>
#include "software/test_util/test_util.h"

TEST(PlayTest, test) {
    auto play = MoveTestPlay();
    auto world = ::Test::TestUtil::createBlankTestingWorld();
    world = ::Test::TestUtil::setFriendlyRobotPositions(world, {
        Point(0, 0),
        Point(1, 0),
        Point(2, 0)
    }, Timestamp::fromSeconds(0));

    Point ball_position = Point(0, 0);
    world = ::Test::TestUtil::setBallPosition(world, ball_position, Timestamp::fromSeconds(0));

    for(unsigned int i = 0; i < 10; i++) {
        ball_position += Vector(1, 0);
        world = ::Test::TestUtil::setBallPosition(world, ball_position, Timestamp::fromSeconds(0));
        play.getTactics(world);
    }
}
