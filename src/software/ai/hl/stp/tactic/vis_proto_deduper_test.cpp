
#include "software/ai/hl/stp/tactic/vis_proto_deduper.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/polygon.h"
#include "software/geom/point.h"

#include <gtest/gtest.h>
#include <memory>
#include <vector>



class VisProtoDeduperTest : public ::testing::Test
{
protected:
    RobotNavigationObstacleFactory obstacle_factory =
        RobotNavigationObstacleFactory(TbotsProto::RobotNavigationObstacleConfig());

    // Helper to extract the list of obstacles from the proto message for easy verification
    std::vector<TbotsProto::Obstacle> getObstaclesFromProto(const TbotsProto::ObstacleList& msg)
    {
        std::vector<TbotsProto::Obstacle> obstacles;
        for (const auto& obs : msg.obstacles())
        {
            obstacles.push_back(obs);
        }
        return obstacles;;
    }

    // Helper to create a unique obstacle based on a position offset
    // This ensures we have distinct geometries to hash.
    ObstaclePtr createTestObstacle(double x, double y)
    {
        auto polygon = Polygon({Point(x, y), Point(x + 1.0, y), Point(x, y + 1.0)});
        return obstacle_factory.createFromShape(polygon);
    }
};

TEST_F(VisProtoDeduperTest, DeduplicatesRepeatedObstacles)
{
    VisProtoDeduper deduper(5);
    TbotsProto::ObstacleList output_msg;

    auto obs1 = createTestObstacle(10, 10);
    std::vector<ObstaclePtr> input = {obs1};

    // First pass: Obstacle is new
    deduper.dedupeAndFill(input, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1);

    // Clear output for next step
    output_msg.Clear();

    // Second pass: Same obstacle passed immediately again
    deduper.dedupeAndFill(input, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 0) << "Should filter out recently sent obstacle";
}

TEST_F(VisProtoDeduperTest, HandlesMixedNewAndOldObstacles)
{
    VisProtoDeduper deduper(5);
    TbotsProto::ObstacleList output_msg;

    auto obs_old = createTestObstacle(10, 10);
    auto obs_new = createTestObstacle(20, 20);

    // Step 1: Send first obstacle
    deduper.dedupeAndFill({obs_old}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1);
    output_msg.Clear();

    // Step 2: Send both. 'obs_old' should be deduped, 'obs_new' should pass.
    deduper.dedupeAndFill({obs_old, obs_new}, output_msg);

    ASSERT_EQ(output_msg.obstacles_size(), 1);
}

TEST_F(VisProtoDeduperTest, WindowEvictionLogic)
{
    // Window size of 2
    // Frame 0: Send A (Stored in queue index 0)
    // Frame 1: Send empty (Stored in queue index 1)
    // Frame 2: Send empty (Stored in queue index 2) -> Window exceeded? 
    // Logic check: if queue.size() > window. 
    // After Frame 0: size 1. 
    // After Frame 1: size 2. 
    // After Frame 2: size 3. (3 > 2, so Frame 0 is evicted).

    VisProtoDeduper deduper(2);
    TbotsProto::ObstacleList output_msg;
    auto obs = createTestObstacle(5, 5);

    deduper.dedupeAndFill({obs}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1);
    output_msg.Clear();

    deduper.dedupeAndFill({}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 0);

    deduper.dedupeAndFill({}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 0);

    deduper.dedupeAndFill({obs}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1) << "Obstacle should be resent after window expiration";
}

TEST_F(VisProtoDeduperTest, ZeroWindowAlwaysSends)
{
    // If window size is 0, it should behave like a pass-through (or evict immediately)
    VisProtoDeduper deduper(0);
    TbotsProto::ObstacleList output_msg;
    auto obs = createTestObstacle(1, 1);

    // Pass 1
    deduper.dedupeAndFill({obs}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1);
    output_msg.Clear();

    // Pass 2 - Should send again because window size is 0 (immediate eviction)
    deduper.dedupeAndFill({obs}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 1);
}

TEST_F(VisProtoDeduperTest, MultipleDistinctObstaclesInOneBatch)
{
    VisProtoDeduper deduper(5);
    TbotsProto::ObstacleList output_msg;

    auto obs1 = createTestObstacle(1, 1);
    auto obs2 = createTestObstacle(2, 2);
    auto obs3 = createTestObstacle(3, 3);

    // Send 3 unique obstacles at once
    deduper.dedupeAndFill({obs1, obs2, obs3}, output_msg);
    EXPECT_EQ(output_msg.obstacles_size(), 3);
    output_msg.Clear();

    // Send 2 old, 1 new
    auto obs4 = createTestObstacle(4, 4);
    deduper.dedupeAndFill({obs1, obs3, obs4}, output_msg);

    ASSERT_EQ(output_msg.obstacles_size(), 1);
}

