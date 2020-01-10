#include "software/ai/hl/stp/tactic/goalie_tactic.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

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
    auto small_rectangle = world.field().friendlyDefenseArea();
    small_rectangle.expand(-0.8);
    auto requested_position = GetParam();
    auto restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, small_rectangle);

    // scaling the restrained position by a slight bit as containsPoint does not count
    // the points right on the edge of the rectangle. For the purposes of the goalie
    // we are okay if the point is right on the edge, or close enough.
    EXPECT_TRUE(small_rectangle.containsPoint((*restrained_position)));
    EXPECT_FALSE(small_rectangle.containsPoint(requested_position));

    // test to make sure that points given inside of the rectangle
    // are not altered and are the same points
    // are constrained inside.
    auto big_rectangle = world.field().friendlyDefenseArea();

    // blow up rectangle to a huge amount, to contain all the points
    big_rectangle.expand(5);
    restrained_position =
        tactic.restrainGoalieInRectangle(requested_position, big_rectangle);

    EXPECT_TRUE(big_rectangle.containsPoint(requested_position));
    EXPECT_TRUE(big_rectangle.containsPoint(*restrained_position));
}

INSTANTIATE_TEST_CASE_P(Positions, GoalieRestrainTest,
                        ::testing::Values(Point(0, 0), Point(1, 2), Point(0, 2),
                                          Point(1, 1), Point(0.5, 1), Point(1, 0.5)));
