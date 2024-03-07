
#include "software/geom/algorithms/end_in_obstacle_sample.h"

#include <include/gtest/gtest.h>

#include <random>

#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/test_util/test_util.h"

static constexpr double MAX_ALLOWABLE_SAMPLE_ERROR = 0.15;

class EndInObstacleSampleTest : public testing::Test
{
   public:
    EndInObstacleSampleTest()
        : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_B)),
          obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()){};
    static bool isSamplePointValid(Point point, std::vector<ObstaclePtr> obstacles)
    {
        double min_distance = MAX_ALLOWABLE_SAMPLE_ERROR + 1;
        for (auto const &obstacle : obstacles)
        {
            double point_distance_from_obstacle =
                distance(point, obstacle->closestPoint(point));
            min_distance = std::min(min_distance, point_distance_from_obstacle);
            // check that the returned point is not inside an obstacle
            if (obstacle->contains(point))
            {
                return false;
            }
        }
        // check that returned point is not too far from the nearest obstacle
        if (!obstacles.empty() && min_distance > MAX_ALLOWABLE_SAMPLE_ERROR)
        {
            return false;
        }
        return true;
    }

   protected:
    std::shared_ptr<World> world;
    RobotNavigationObstacleFactory obstacle_factory;
};


TEST_F(EndInObstacleSampleTest, test_end_outside_field_boundary)
{
    Field field              = world->field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> field_boundary =
        obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE, field);
    obstacles.insert(obstacles.end(), field_boundary.begin(), field_boundary.end());

    Point destination(4.9, 2);
    std::optional<Point> sample_point =
        endInObstacleSample(obstacles, destination, navigable_area);

    if (!sample_point.has_value() ||
        !EndInObstacleSampleTest::isSamplePointValid(sample_point.value(), obstacles))
    {
        FAIL();
    }
}

TEST_F(EndInObstacleSampleTest, test_end_in_defense_area)
{
    Field field              = world->field();
    Rectangle navigable_area = field.fieldBoundary();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> friendly_defense_area =
        obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(),
                     friendly_defense_area.end());

    Point destination(-4, 0.5);
    std::optional<Point> sample_point =
        endInObstacleSample(obstacles, destination, navigable_area);

    if (!sample_point.has_value() ||
        !EndInObstacleSampleTest::isSamplePointValid(sample_point.value(), obstacles))
    {
        FAIL();
    }
}

TEST_F(EndInObstacleSampleTest, test_end_outside_navigable_area)
{
    Field field              = world->field();
    Rectangle navigable_area = field.fieldBoundary();
    std::vector<ObstaclePtr> obstacles;

    Point destination(0, 5.2);
    std::optional<Point> sample_point =
        endInObstacleSample(obstacles, destination, navigable_area);

    if (!sample_point.has_value() ||
        !EndInObstacleSampleTest::isSamplePointValid(sample_point.value(), obstacles))
    {
        FAIL();
    }
}

TEST_F(EndInObstacleSampleTest, test_end_not_in_obstacle)
{
    Field field              = world->field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    Point destination(1, 1);
    std::optional<Point> end_point =
        endInObstacleSample(obstacles, destination, navigable_area);

    if (end_point.has_value())
    {
        ASSERT_EQ(destination, end_point.value());
    }
    else
    {
        FAIL();
    }
}

TEST_F(EndInObstacleSampleTest, test_sampling_performance)
{
    Field field              = world->field();
    Rectangle navigable_area = field.fieldLines();
    std::vector<ObstaclePtr> obstacles;

    std::vector<ObstaclePtr> field_boundary =
        obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE, field);
    obstacles.insert(obstacles.end(), field_boundary.begin(), field_boundary.end());

    std::vector<ObstaclePtr> friendly_defense_area =
        obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, field);
    obstacles.insert(obstacles.end(), friendly_defense_area.begin(),
                     friendly_defense_area.end());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_x(navigable_area.xMin(), navigable_area.xMax());
    std::uniform_real_distribution<> dist_y(navigable_area.yMin(), navigable_area.yMax());

    std::vector<Point> points_in_obstacles;
    // find a random test group of 40 points inside obstacles to test on
    while (points_in_obstacles.size() < 40)
    {
        Point sample_point(dist_x(gen), dist_y(gen));
        bool point_in_obstacle = false;
        for (auto const &obstacle : obstacles)
        {
            if (obstacle->contains(sample_point))
            {
                point_in_obstacle = true;
                break;
            }
        }
        if (point_in_obstacle)
        {
            // add point to test group if it is inside an obstacle
            points_in_obstacles.push_back(sample_point);
        }
    }

    // test sampling on each point in the test group
    for (auto const &point : points_in_obstacles)
    {
        std::optional<Point> sample_point =
            endInObstacleSample(obstacles, point, navigable_area, 6);
        if (!sample_point.has_value() ||
            !EndInObstacleSampleTest::isSamplePointValid(sample_point.value(), obstacles))
        {
            FAIL();
        }
    }
}
