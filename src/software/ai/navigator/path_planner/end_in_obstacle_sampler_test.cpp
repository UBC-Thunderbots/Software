
#include <gtest/gtest.h>
#include "software/test_util/test_util.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_planner/end_in_obstacle_sampler.h"


class TestEndInObstacleSampler : public testing::Test {
public:
    TestEndInObstacleSampler()
    : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_A)),
      obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()),
      sampler(){};
protected:
    World world;
    RobotNavigationObstacleFactory obstacle_factory;
    EndInObstacleSampler sampler;
};


TEST_F(TestEndInObstacleSampler, test_end_outside_field_boundary)
{
    Field field = world.field();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> field_boundary = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE, field);
    obstacles.insert(obstacles.end(), field_boundary.begin(), field_boundary.end());

    Point destination =
}

TEST_F(TestEndInObstacleSampler, test_end_in_defense_area)
{
    Field field = world.field();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> friendly_defense_area = obstacle_factory.createStaticObstaclesFromMotionConstraint(TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(), friendly_defense_area.end());

    Point destination =

}