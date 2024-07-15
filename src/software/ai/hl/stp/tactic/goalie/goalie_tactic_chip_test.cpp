#include <gtest/gtest.h>

#include "software/ai/hl/stp/tactic/goalie/goalie_fsm.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class GoalieTacticChipTest
    :  // Params: enemy robot positions, expected chip region to avoid
       public testing::TestWithParam<std::tuple<std::vector<Point>, Rectangle>>
{
};

TEST_P(GoalieTacticChipTest, test_find_good_chip_targets)
{
    TbotsProto::GoalieTacticConfig goalie_tactic_config;

    std::vector<Point> enemy_robot_positions = std::get<0>(GetParam());
    Rectangle expected_chip_region_to_avoid  = std::get<1>(GetParam());

    std::shared_ptr<World> world = TestUtil::createBlankTestingWorld();
    TestUtil::setEnemyRobotPositions(world, enemy_robot_positions, Timestamp());
    world->updateBall(Ball(world->field().friendlyGoalCenter(), Vector(), Timestamp()));

    Point chip_target = GoalieFSM::findGoodChipTarget(*world, goalie_tactic_config);

    // Chip target should be in region in front of friendly defense area
    EXPECT_TRUE(contains(Rectangle(world->field().friendlyCornerPos() +
                                       Vector(world->field().defenseAreaXLength(), 0),
                                   world->field().enemyCornerNeg()),
                         chip_target));

    EXPECT_FALSE(contains(expected_chip_region_to_avoid, chip_target));
}

INSTANTIATE_TEST_CASE_P(
    ChipEnvironment, GoalieTacticChipTest,
    ::testing::Values(
        // Wall of enemies in enemy half
        std::make_tuple<std::vector<Point>, Rectangle>(
            {Point(2, 2.5), Point(2, 2), Point(2, 1.5), Point(2, 1), Point(2, 0.5),
             Point(2, 0), Point(2, -0.5), Point(2, -1), Point(2, -1.5), Point(2, -2),
             Point(2, -2.5)},
            Field::createSSLDivisionBField().enemyHalf()),
        // Wall of enemies in friendly half
        std::make_tuple<std::vector<Point>, Rectangle>(
            {Point(-2, 2.5), Point(-2, 2), Point(-2, 1.5), Point(-2, 1), Point(-2, 0.5),
             Point(-2, 0), Point(-2, -0.5), Point(-2, -1), Point(-2, -1.5), Point(-2, -2),
             Point(-2, -2.5)},
            Field::createSSLDivisionBField().friendlyHalf()),
        // Wall of enemies in friendlyPositiveYQuadrant
        std::make_tuple<std::vector<Point>, Rectangle>(
            {Point(2, 2.5), Point(2, 2), Point(2, 1.5), Point(2, 1), Point(2, 0.5),
             Point(2, 0)},
            Field::createSSLDivisionBField().friendlyPositiveYQuadrant()),
        // Wall of enemies in friendlyNegativeYQuadrant
        std::make_tuple<std::vector<Point>, Rectangle>(
            {Point(2, -2.5), Point(2, -2), Point(2, -1.5), Point(2, -1), Point(2, -0.5),
             Point(2, 0)},
            Field::createSSLDivisionBField().friendlyNegativeYQuadrant())));
