
#include <include/gtest/gtest.h>
#include <random>
#include "software/test_util/test_util.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/algorithms/end_in_obstacle_sample.h"


class TestEndInObstacleSampler : public testing::Test {
public:
    TestEndInObstacleSampler()
    : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_B)),
      obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()) {};
protected:
    World world;
    RobotNavigationObstacleFactory obstacle_factory;
};


TEST_F(TestEndInObstacleSampler, test_end_outside_field_boundary)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> field_boundary = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE, field);
    obstacles.insert(obstacles.end(), field_boundary.begin(), field_boundary.end());

    Point destination(4.9, 2);
    std::optional<Point> end_point = endInObstacleSample(obstacles, destination, navigable_area);

    if (end_point.has_value()) {
        for (auto const &obstacle : obstacles) {
            ASSERT_FALSE(obstacle->contains(end_point.value()));
        }
    } else {
        FAIL();
    }
}

TEST_F(TestEndInObstacleSampler, test_end_in_defense_area)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldBoundary();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> friendly_defense_area = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(), friendly_defense_area.end());

    Point destination(-4, 0.5);
    std::optional<Point> end_point = endInObstacleSample(obstacles, destination, navigable_area);

    if (end_point.has_value()) {
        for (auto const &obstacle : obstacles) {
            ASSERT_FALSE(obstacle->contains(end_point.value()));
        }
    } else {
        FAIL();
    }
}

TEST_F(TestEndInObstacleSampler, test_end_not_in_obstacle)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    Point destination(1, 1);
    std::optional<Point> end_point = endInObstacleSample(obstacles, destination, navigable_area);

    if (end_point.has_value()) {
        ASSERT_EQ(destination, end_point.value());
    } else {
        FAIL();
    }
}

TEST_F(TestEndInObstacleSampler, test_sampling_performance)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> field_boundary = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE, field);
    obstacles.insert(obstacles.end(), field_boundary.begin(), field_boundary.end());

    std::vector<ObstaclePtr> friendly_defense_area = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(), friendly_defense_area.end());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(navigable_area.xMin(), navigable_area.xMax());
    std::uniform_real_distribution<> dist_y(navigable_area.yMin(), navigable_area.yMax());

    std::vector<Point> points_in_obstacles;
    while (points_in_obstacles.size() < 40) {
        Point sample_point(dist_x(gen), dist_y(gen));
        bool point_in_obstacle = false;
        for (auto const &obstacle : obstacles) {
            if (obstacle->contains(sample_point)) {
                point_in_obstacle = true;
                break;
            }
        }
        if (point_in_obstacle) {
            points_in_obstacles.push_back(sample_point);
        }
    }

    for (auto const &point : points_in_obstacles) {
        std::optional<Point> end_point = endInObstacleSample(obstacles, point, navigable_area, 6);
        if (!end_point.has_value()) {
            FAIL();
        }
        for (auto const &obstacle : obstacles) {
            if (obstacle->contains(end_point.value())) {
                FAIL();
            }
        }
    }
}