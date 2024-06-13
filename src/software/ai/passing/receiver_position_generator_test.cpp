#include "software/ai/passing/receiver_position_generator.hpp"

#include <gtest/gtest.h>

#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/test_util/test_util.h"

class ReceiverPositionGeneratorTest : public testing::Test
{
   public:
    ReceiverPositionGeneratorTest()
        : receiver_position_generator(ReceiverPositionGenerator<EighteenZoneId>(
              std::make_shared<const EighteenZonePitchDivision>(
                  Field::createSSLDivisionBField()),
              passing_config)),
          world(::TestUtil::createBlankTestingWorld())
    {
    }

   protected:
    /**
     * Verify that the receiving position is open and not blocked by any enemy robots
     * @param receiving_positions A list of receiving positions to verify
     */
    void verifyReceivingPositionIsOpen(const std::vector<Point> &receiving_positions)
    {
        for (const Point &receiving_position : receiving_positions)
        {
            Segment pass_segment(world->ball().position(), receiving_position);
            for (const Robot &robot : world->enemyTeam().getAllRobots())
            {
                EXPECT_FALSE(intersects(Circle(robot.position(), ROBOT_MAX_RADIUS_METERS),
                                        pass_segment))
                    << "Receiving position " << receiving_position
                    << " intersects with enemy robot at " << robot.position();
            }
        }
    }

    /**
     * Get the best receiving positions from the receiver position generator after
     * running the generator 100 times
     * @param num_positions The number of best receiving positions to get
     * @return A list of the best receiving positions
     */
    std::vector<Point> getConvergedBestReceivingPositions(int num_positions)
    {
        std::vector<Point> best_receiving_positions;
        for (int i = 0; i < 100; ++i)
        {
            best_receiving_positions =
                receiver_position_generator.getBestReceivingPositions(*world,
                                                                      num_positions);
        }
        return best_receiving_positions;
    }

    TbotsProto::PassingConfig passing_config;
    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator;
    // Empty world
    std::shared_ptr<World> world;
};

TEST_F(ReceiverPositionGeneratorTest, test_single_receiver)
{
    ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(
        world, {{1, -2}, {1, 0}, {1, 2}, {3, 0.1}, {3, -0.1}, {4.4, 0}},
        Timestamp::fromSeconds(0));
    ::TestUtil::setFriendlyRobotPositions(world, {Point(2, 0)},
                                          Timestamp::fromSeconds(0));

    int num_positions = 1;
    std::vector<Point> best_receiving_positions =
        getConvergedBestReceivingPositions(num_positions);

    ASSERT_EQ(best_receiving_positions.size(), num_positions);
    verifyReceivingPositionIsOpen(best_receiving_positions);
}

TEST_F(ReceiverPositionGeneratorTest, test_multiple_receiver)
{
    ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(
        world, {{1, -2}, {1, 0}, {1, 2}, {3, 0.1}, {3, -0.1}, {4.4, 0}},
        Timestamp::fromSeconds(0));
    ::TestUtil::setFriendlyRobotPositions(world, {Point(2, -2), Point(2, 0), Point(2, 2)},
                                          Timestamp::fromSeconds(0));

    int num_positions = 3;
    std::vector<Point> best_receiving_positions =
        getConvergedBestReceivingPositions(num_positions);

    ASSERT_EQ(best_receiving_positions.size(), num_positions);
    verifyReceivingPositionIsOpen(best_receiving_positions);
}

TEST_F(ReceiverPositionGeneratorTest, test_half_of_field_blocked)
{
    // +y half of the field is blocked by enemy robots, verify that the receiving
    // positions are all in the -y half of the field
    ::TestUtil::setBallPosition(world, Point(0, 0), Timestamp::fromSeconds(0));
    ::TestUtil::setFriendlyRobotPositions(world, {Point(2, -2), Point(2, 2)},
                                          Timestamp::fromSeconds(0));

    std::vector<Point> enemy_positions = {Point(4.4, 0)};
    for (int i = 0; i < 6; i++)
    {
        enemy_positions.push_back(Point(3.5, 0.3 * i));
    }
    ::TestUtil::setEnemyRobotPositions(world, enemy_positions, Timestamp::fromSeconds(0));

    int num_positions = 2;
    std::vector<Point> best_receiving_positions =
        getConvergedBestReceivingPositions(num_positions);

    ASSERT_EQ(best_receiving_positions.size(), num_positions);
    for (const Point &receiving_position : best_receiving_positions)
    {
        EXPECT_LT(receiving_position.y(), 0)
            << "Receiving position " << receiving_position
            << " is not in the -y half of the field";
    }
    verifyReceivingPositionIsOpen(best_receiving_positions);
}

TEST_F(ReceiverPositionGeneratorTest, test_receiving_position_rating_not_degrading)
{
    // Verify that with each iteration of the receiver position generator over
    // the same world, the rating of the receiver positions does not degrade
    Point ball_pos(0, 0);
    ::TestUtil::setBallPosition(world, ball_pos, Timestamp::fromSeconds(0));
    ::TestUtil::setEnemyRobotPositions(
        world, {{2, -1}, {2, 0}, {2, 1}, {3.5, 0.1}, {3.5, -0.1}, {4.4, 0}},
        Timestamp::fromSeconds(0));
    ::TestUtil::setFriendlyRobotPositions(world, {Point(2, -2), Point(2, 0), Point(2, 2)},
                                          Timestamp::fromSeconds(0));

    double prev_score = 0;
    for (int i = 0; i < 10; ++i)
    {
        std::vector<Point> best_receiving_positions =
            receiver_position_generator.getBestReceivingPositions(*world, 1);
        double score = rateReceivingPosition(
            *world,
            Pass::fromDestReceiveSpeed(ball_pos, best_receiving_positions[0],
                                       passing_config),
            passing_config);
        EXPECT_GE(score, prev_score);
        prev_score = score;
    }
}
