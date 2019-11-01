#include "software/ai/navigator/path_manager/obstacle_path_manager.h"
#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

#include <gtest/gtest.h>

#include "software/geom/point.h"

TEST(TestObstaclePathManager, test_no_obostacles)
{
    Point start{0, 0}, dest{1, 1};
    auto path_manager = std::make_unique<ObstaclePathManager>(std::make_unique<StraightLinePathPlanner>());
    std::vector<Obstacle> obstacles;
    std::vector<Obstacle> avoid_area_obstacles;

    Rectangle navigable_area = Rectangle(Point(0, 0), 0, 0);
    std::set<PathObjective> path_objectives;
    PathObjective po1(Point(1,3), Point(2,3), 2.0, avoid_area_obstacles, 1);
    PathObjective po2(Point(2,4), Point(2,5), 1.0, avoid_area_obstacles, 2);
    path_objectives.insert(po1);
    path_objectives.insert(po2);

    auto paths = path_manager->getManagedPaths(path_objectives, navigable_area, obstacles);

    auto path1 = paths[po1];
    auto path2 = paths[po2];

    EXPECT_TRUE(path1 !=std::nullopt);
    EXPECT_TRUE(path2 !=std::nullopt);

    std::vector<Point> path_points1 = path1->getKnots();
    EXPECT_EQ(path_points1.front(), po1.start);
    EXPECT_EQ(path_points1.back(), po1.end);

    std::vector<Point> path_points2 = path2->getKnots();
    EXPECT_EQ(path_points2.front(), po2.start);
    EXPECT_EQ(path_points2.back(), po2.end);
}
