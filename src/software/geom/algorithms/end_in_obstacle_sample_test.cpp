
#include <include/gtest/gtest.h>
#include "software/test_util/test_util.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/algorithms/end_in_obstacle_sample.h"


class TestEndInObstacleSampler : public testing::Test {
public:
    TestEndInObstacleSampler()
    : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_A)),
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
    Point end_point = endInObstacleSample(obstacles, destination, navigable_area);
    LOG(WARNING) << "test_end_outside_field_boundary " << end_point.x() << " " << end_point.y();

    for (auto const &obstacle : field_boundary) {
        ASSERT_FALSE(obstacle->contains(end_point));
    }
}

TEST_F(TestEndInObstacleSampler, test_end_in_defense_area)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldBoundary();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> friendly_defense_area = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(), friendly_defense_area.end());

    Point destination(4, 0.5);
    Point end_point = endInObstacleSample(obstacles, destination, navigable_area);
    LOG(WARNING) << "test_end_in_defense_area " << end_point.x() << " " << end_point.y();

    for (auto const &obstacle : friendly_defense_area) {
        ASSERT_FALSE(obstacle->contains(end_point));
    }
}

TEST_F(TestEndInObstacleSampler, test_end_not_in_obstacle)
{
    Field field = world.field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    Point destination(1, 1);
    Point end_point = endInObstacleSample(obstacles, destination, navigable_area);
    LOG(WARNING) << "test_end_not_in_obstacle " << end_point.x() << " " << end_point.y();

    ASSERT_EQ(destination, end_point);
}