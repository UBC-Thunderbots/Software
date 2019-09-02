#include "ai/hl/stp/tactic/goalie_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/move_intent.h"
#include "test/test_util/test_util.h"

// The following tests will make sure the goalie stays in the requested
// deflated defense area when best positioning to defend the ball.
// The diagram below shows the requested position (R) by the goalie to where
// it should be defending, the restrain function returns the actual position (A)
// where the goalie is allowed to stay.
//
//
//    |
//    |defense
//    +-----------------+
//    |restrain         |
//    +-------+         |
// +--+       |         |
// |  |       |    R    |
// |  |      A|         |
// |  |       |         |
// |  |       |         |
// |  |       |         |
// |  |       |         |
// +--+       |         |
//    +-------+         |
//    |                 |
//    +-----------------+
//    |
//    |

class GoalieRestrainTest : public ::testing::TestWithParam<Point>
{
};

TEST_P(GoalieRestrainTest, goalie_position_safe)
{
    World world         = ::Test::TestUtil::createBlankTestingWorld();
    GoalieTactic tactic = GoalieTactic(world.ball(), world.field(), world.friendlyTeam(),
                                       world.enemyTeam());

    // test to make sure that points given outside of the rectangle
    // are constrained inside
    auto small_rectangle    = Rectangle(Point(0, 0), 1, 1);
    auto requested_position = GetParam();
    auto restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, small_rectangle);

    EXPECT_FALSE(small_rectangle.containsPoint(requested_position));
    EXPECT_TRUE(small_rectangle.containsPoint(*restrained_position));

    // test to make sure that points given inside of the rectangle
    // are not altered and are the same points
    // are constrained inside
    auto big_rectangle = Rectangle(Point(0, 0), 4, 4);
    restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, big_rectangle);

    EXPECT_EQ(*restrained_position, requested_position);
    EXPECT_TRUE(big_rectangle.containsPoint(requested_position));
    EXPECT_TRUE(big_rectangle.containsPoint(*restrained_position));
}

INSTANTIATE_TEST_CASE_P(Positions, GoalieRestrainTest,
                        ::testing::Values(Point(2, 0), Point(0, 2), Point(2, 3),
                                          Point(2, 2), Point(3, 3), Point(4, 2)));
