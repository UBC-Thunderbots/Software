/**
 * This file contains the unit tests for evaluation functions
 * in calc_best_shot.cpp
 */

#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include <gtest/gtest.h>

#include "geom/util.h"
#include "shared/constants.h"

TEST(CalcBestShotTest, calc_best_shot_with_vector_of_points_test)
{
    Field f                      = Field(10, 8, 10, 8, 3, 1, 1);
    std::vector<Point> obstacles = {Point(2, -3), Point(6, 2), Point(1, 4)};
    const Point p                = Point(0, 0);
    double radius                = 1.0;
    const Point p1               = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
    const Point p2               = Point(f.length() / 2.0, f.goalWidth() / 2.0);
    EXPECT_EQ(angleSweepCircles(p, p1, p2, obstacles, radius * ROBOT_MAX_RADIUS_METERS),
              Evaluation::calcBestShot(f, obstacles, p, radius));
}

TEST(CalcBestShotTest, calc_best_shot_all_with_vector_of_points_test)
{
    Field f                      = Field(10, 8, 10, 8, 3, 1, 1);
    std::vector<Point> obstacles = {Point(2, -3), Point(6, 2), Point(1, 4)};
    const Point p                = Point(0, 0);
    double radius                = 1.0;
    const Point p1               = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
    const Point p2               = Point(f.length() / 2.0, f.goalWidth() / 2.0);
    EXPECT_EQ(
        angleSweepCirclesAll(p, p1, p2, obstacles, radius * ROBOT_MAX_RADIUS_METERS),
        Evaluation::calcBestShotAll(f, obstacles, p, radius));
}

TEST(CalcBestShotTest, calc_best_shot_with_world_test)
{
    Field f = Field(10, 8, 10, 8, 3, 1, 1);
    Ball b  = Ball(Point(3, 0), Vector(0, 0), Timestamp());
    Robot r = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                    Timestamp());
    double radius = 1.0;

    std::vector<Robot> friendly = {r,
                                   Robot(1, Point(2, 1), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp()),
                                   Robot(2, Point(4, 1), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp())};
    std::vector<Robot> enemy    = {Robot(0, Point(6, 2), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp()),
                                Robot(1, Point(7, -1), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp())};

    std::vector<Point> obstacles;
    obstacles.reserve(friendly.size() + enemy.size());
    for (const Robot i : friendly)
    {
        if (i != r)
        {
            obstacles.emplace_back(i.position());
        }
    }
    for (const Robot i : enemy)
    {
        obstacles.emplace_back(i.position());
    }

    Team friendly_team = Team(Duration());
    friendly_team.updateRobots(friendly);
    Team enemy_team = Team(Duration());
    enemy_team.updateRobots(enemy);
    World w = World(f, b, friendly_team, enemy_team);

    EXPECT_EQ(Evaluation::calcBestShot(f, obstacles, r.position(), radius),
              Evaluation::calcBestShot(w, r, radius));
}

TEST(CalcBestShotTest, calc_best_shot_with_world_no_available_shot_test)
{
    Field f = Field(10, 8, 10, 8, 5, 1, 1);
    Ball b  = Ball(Point(3, 0), Vector(0, 0), Timestamp());
    Robot r = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                    Timestamp());
    double radius = 1.0;

    std::vector<Robot> friendly = {r,
                                   Robot(1, Point(2.1, 0), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp()),
                                   Robot(2, Point(4, 1), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp())};


    std::vector<Robot> enemy = {Robot(0, Point(6, 2), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp()),
                                Robot(1, Point(7, -1), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp())};

    std::vector<Point> obstacles;
    obstacles.reserve(friendly.size() + enemy.size());
    for (const Robot i : friendly)
    {
        if (i != r)
        {
            obstacles.emplace_back(i.position());
        }
    }
    for (const Robot i : enemy)
    {
        obstacles.emplace_back(i.position());
    }

    Team friendly_team = Team(Duration());
    friendly_team.updateRobots(friendly);
    Team enemy_team = Team(Duration());
    enemy_team.updateRobots(enemy);
    World w = World(f, b, friendly_team, enemy_team);

    EXPECT_EQ(Evaluation::calcBestShot(f, obstacles, r.position(), radius),
              Evaluation::calcBestShot(w, r, radius));
}

TEST(CalcBestShotTest, calc_best_shot_all_with_world_test)
{
    Field f = Field(10, 8, 10, 8, 5, 1, 1);
    Ball b  = Ball(Point(3, 0), Vector(0, 0), Timestamp());
    Robot r = Robot(0, Point(2, 0), Vector(0, 0), Angle::zero(), AngularVelocity::zero(),
                    Timestamp());
    const Point p1 = Point(f.length() / 2.0, -f.goalWidth() / 2.0);
    const Point p2 = Point(f.length() / 2.0, f.goalWidth() / 2.0);
    double radius  = 1.0;

    std::vector<Robot> friendly = {r,
                                   Robot(1, Point(2, 1), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp()),
                                   Robot(2, Point(4, 1), Vector(0, 0), Angle::zero(),
                                         AngularVelocity::zero(), Timestamp())};

    std::vector<Robot> enemy = {Robot(0, Point(6, 2), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp()),
                                Robot(1, Point(7, -1), Vector(0, 0), Angle::zero(),
                                      AngularVelocity::zero(), Timestamp())};
    std::vector<Point> obstacles;
    obstacles.reserve(friendly.size() + enemy.size());
    for (const Robot i : friendly)
    {
        if (i != r)
        {
            obstacles.emplace_back(i.position());
        }
    }
    for (const Robot i : enemy)
    {
        obstacles.emplace_back(i.position());
    }

    Team friendly_team = Team(Duration());
    friendly_team.updateRobots(friendly);
    Team enemy_team = Team(Duration());
    enemy_team.updateRobots(enemy);
    World w = World(f, b, friendly_team, enemy_team);

    EXPECT_EQ(Evaluation::calcBestShotAll(f, obstacles, r.position(), radius),
              Evaluation::calcBestShotAll(w, r, radius));
}