#include "software/ai/navigator/path_planner/global_path_planner_factory.h"

#include <chrono>

#include "proto/parameters.pb.h"
#include "software/test_util/path_planning_test_util.h"
#include "software/test_util/test_util.h"

class TestGlobalPathPlanner : public testing::Test
{
   public:
    TestGlobalPathPlanner()
        : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_A)),
          gpp(TbotsProto::RobotNavigationObstacleConfig(), world.field()),
          obstacle_factory(TbotsProto::RobotNavigationObstacleConfig())
    {
    }

   protected:
    World world;
    GlobalPathPlannerFactory gpp;
    RobotNavigationObstacleFactory obstacle_factory;
};

TEST_F(TestGlobalPathPlanner, test_no_motion_constraints)
{
    Point start{1, 2}, dest{3, -2};

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner({});
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    Rectangle bounding_box({0.9, -2.1}, {3.1, 2.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
}

TEST_F(TestGlobalPathPlanner,
       test_both_defence_areas_blocked_starting_and_ending_in_blocked_areas)
{
    Point start = world.field().enemyGoalCenter();
    Point dest  = world.field().friendlyGoalCenter();

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    EXPECT_EQ(start, path.front());

    EXPECT_LE(path.value().back().x(), world.field().enemyDefenseArea().negXNegYCorner().x());
    EXPECT_NEAR(path.value().back().y(), 0, 0.1);

    Rectangle bounding_box(world.field().enemyDefenseArea().posXPosYCorner(),
                           world.field().friendlyDefenseArea().negXNegYCorner());
    TestUtil::checkPathDoesNotExceedBoundingBox(path.value(), bounding_box);
    // since the starting point is inside an obstacle, we need to consider the path from
    // the second point onwards
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path_points.begin() + 1, path_points.end()}, obstacles);
}

TEST_F(
    TestGlobalPathPlanner,
    test_both_defense_areas_and_centre_circle_blocked_starting_and_ending_in_blocked_areas)
{
    Point start{-5.6, 0}, dest{5.6, 0.1};

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::CENTER_CIRCLE,
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path.value();

    EXPECT_EQ(path.value().front(), start);

    EXPECT_LE(path.value().back().x(),
              world.field().enemyDefenseArea().negXNegYCorner().x());
    EXPECT_NEAR(path.value().back().y(), dest.y(), planner->getResolution());

    Rectangle bounding_box(world.field().enemyDefenseArea().posXPosYCorner(),
                           world.field().friendlyDefenseArea().negXNegYCorner());
    TestUtil::checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    // since the starting point is inside an obstacle, we need to consider the path from
    // the second point onwards

    // we are differentiating between defense area obstacles and the center circle
    // obstacles because the lost resolution means that we interfere with the center
    // circle while path planning (but not to a problematic degree). This won't be a
    // problem since obstacles are inflated by the RobotNavigationObstacleFactory
    std::vector<ObstaclePtr> defense_area_obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
             TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA},
            world.field());
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path_points.begin() + 1, path_points.end()}, defense_area_obstacles);
    std::vector<Polygon> centre_circle_obstacle = {
        Rectangle(Point(-0.5, -0.5), Point(0.5, 0.5))};
    TestUtil::checkPathDoesNotIntersectObstacle(path_points, centre_circle_obstacle);
}

TEST_F(TestGlobalPathPlanner,
       test_enemy_half_and_centre_circle_blocked_starting_in_blocked_area)
{
    Point start{3, 3}, dest{-3, -3};

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::ENEMY_HALF,
        TbotsProto::MotionConstraint::CENTER_CIRCLE,
        TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(start, path.value().front());
    EXPECT_LE(distance(path.value().back(), dest), planner->getResolution());

    Rectangle bounding_box({-3.1, -3.1}, {3.1, 3.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    // Expect the second point to be outside the obstacle
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path_points.begin() + 1, path_points.end()}, obstacles);
}

TEST_F(TestGlobalPathPlanner, test_enemy_half_blocked_starting_and_ending_in_blocked_area)
{
    Point start{2, 1}, dest{0, 0};

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::ENEMY_HALF,
        TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(start, path.value().front());

    EXPECT_LE(dest.x(), 0);
    EXPECT_NEAR(dest.y(), 0, planner->getResolution());

    Rectangle bounding_box({-0.3, -0.1}, {2.1, 1.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    // Expect the second point to be outside the obstacle
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path_points.begin() + 1, path_points.end()}, obstacles);
}

TEST_F(TestGlobalPathPlanner, test_friendly_half_blocked_starting_in_blocked_area)
{
    Point start{-3, -3}, dest{4, 1};

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::FRIENDLY_HALF,
        TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::AVOID_FIELD_BOUNDARY_ZONE};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    Rectangle bounding_box({-3.1, -3.1}, {4.1, 1.1});
    TestUtil::checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    // Expect the second point to be outside the obstacle
    TestUtil::checkPathDoesNotIntersectObstacle(
        {path_points.begin() + 1, path_points.end()}, obstacles);
}

TEST_F(TestGlobalPathPlanner, test_leave_the_field)
{
    Point start{1, 2};
    Point dest = world.field().friendlyCornerPos() + Vector(0, 0.1);

    std::set<TbotsProto::MotionConstraint> constraints = {
        TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA,
        TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA};
    std::vector<ObstaclePtr> obstacles =
        obstacle_factory.createStaticObstaclesFromMotionConstraints(constraints,
                                                                    world.field());

    std::shared_ptr<const EnlsvgPathPlanner> planner = gpp.getPathPlanner(constraints);
    auto path                                        = planner->findPath(start, dest);

    EXPECT_TRUE(path != std::nullopt);

    std::vector<Point> path_points = path->getKnots();

    EXPECT_EQ(start, path.value().front());
    EXPECT_EQ(dest, path.value().back());

    Rectangle bounding_box(start, dest);
    TestUtil::checkPathDoesNotExceedBoundingBox(path_points, bounding_box);
    TestUtil::checkPathDoesNotIntersectObstacle(path_points, obstacles);
}
