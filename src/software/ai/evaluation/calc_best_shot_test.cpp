#include "software/ai/evaluation/calc_best_shot.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot_impl.h"
#include "software/test_util/test_util.h"

TEST(CalcBestShotTest, calc_best_shot_on_enemy_goal_with_no_obstacles)
{
    World world          = ::TestUtil::createBlankTestingWorld();
    Team team            = Team(Duration::fromSeconds(1));
    Robot shooting_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    auto result = calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(), shooting_robot.position(), TeamType::ENEMY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result->getPointToShootAt(),
                                               world.field().enemyGoalCenter(), 0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 12, 5);
}

TEST(CalcBestShotTest, calc_best_shot_on_friendly_goal_with_no_obstacles)
{
    World world          = ::TestUtil::createBlankTestingWorld();
    Team team            = Team(Duration::fromSeconds(1));
    Robot shooting_robot = Robot(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                                 AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    auto result = calcBestShotOnGoal(world.field(), world.friendlyTeam(),
                                     world.enemyTeam(), shooting_robot.position(),
                                     TeamType::FRIENDLY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result->getPointToShootAt(),
                                               world.field().friendlyGoalCenter(), 0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 12, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_enemy_goal_with_obstacles_and_no_obstacles_being_ignored)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoalCenter(), Point(2.5, 0.7), Point(-1, -1)},
        Timestamp::fromSeconds(0));

    auto result =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           shooting_robot.position(), TeamType::ENEMY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        result->getPointToShootAt(), Point(world.field().enemyGoalCenter().x(), -0.3),
        0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_obstacles_and_no_obstacles_being_ignored)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().friendlyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateEnemyTeamState(team);

    world = ::TestUtil::setFriendlyRobotPositions(
        world, {world.field().friendlyGoalCenter(), Point(-2.5, -0.7), Point(1, 1)},
        Timestamp::fromSeconds(0));

    auto result = calcBestShotOnGoal(world.field(), world.friendlyTeam(),
                                     world.enemyTeam(), shooting_robot.position(),
                                     TeamType::FRIENDLY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        result->getPointToShootAt(), Point(world.field().friendlyGoalCenter().x(), -0.3),
        0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_enemy_goal_with_obstacles_and_some_obstacles_being_ignored)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot friendly_blocking_robot =
        Robot(1, Point(1.3, world.field().enemyGoalpostNeg().y() - 0.05), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::TestUtil::setEnemyRobotPositions(
        world, {world.field().enemyGoalCenter(), Point(2.5, 0.7), Point(-1, -1)},
        Timestamp::fromSeconds(0));

    auto result =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           shooting_robot.position(), TeamType::ENEMY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        result->getPointToShootAt(), Point(world.field().enemyGoalCenter().x(), -0.3),
        0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 6, 5);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_obstacles_and_some_obstacles_being_ignored)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().friendlyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot enemy_blocking_robot =
        Robot(1, Point(-1.3, world.field().enemyGoalpostNeg().y() - 0.05), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateEnemyTeamState(team);

    world = ::TestUtil::setFriendlyRobotPositions(
        world, {world.field().friendlyGoalCenter(), Point(-2.5, -0.7), Point(1, 1)},
        Timestamp::fromSeconds(0));

    auto result = calcBestShotOnGoal(world.field(), world.friendlyTeam(),
                                     world.enemyTeam(), shooting_robot.position(),
                                     TeamType::FRIENDLY, {shooting_robot});

    // We expect to be able to find a shot
    ASSERT_TRUE(result);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        result->getPointToShootAt(), Point(world.field().friendlyGoalCenter().x(), -0.3),
        0.05));
    EXPECT_NEAR(result->getOpenAngle().toDegrees(), 6, 5);
}

TEST(CalcBestShotTest, calc_best_shot_on_enemy_goal_with_all_shots_blocked_by_obstacles)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::TestUtil::setEnemyRobotPositions(
        world, {shooting_robot.position() + Vector(ROBOT_MAX_RADIUS_METERS * 2, 0)},
        Timestamp::fromSeconds(0));

    auto result =
        calcBestShotOnGoal(world.field(), world.friendlyTeam(), world.enemyTeam(),
                           shooting_robot.position(), TeamType::ENEMY, {shooting_robot});

    // We should not be able to find a shot
    EXPECT_FALSE(result);
}

TEST(CalcBestShotTest,
     calc_best_shot_on_friendly_goal_with_all_shots_blocked_by_obstacles)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Team team   = Team(Duration::fromSeconds(1));
    Robot shooting_robot =
        Robot(0, Point(-1, world.field().enemyGoalpostNeg().y()), Vector(0, 0),
              Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    team.updateRobots({shooting_robot});
    world.updateFriendlyTeamState(team);

    world = ::TestUtil::setEnemyRobotPositions(
        world, {shooting_robot.position() - Vector(ROBOT_MAX_RADIUS_METERS * 2, 0)},
        Timestamp::fromSeconds(0));

    auto result = calcBestShotOnGoal(world.field(), world.friendlyTeam(),
                                     world.enemyTeam(), shooting_robot.position(),
                                     TeamType::FRIENDLY, {shooting_robot});

    // We should not be able to find a shot
    EXPECT_FALSE(result);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_unblocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().enemyGoalCenter() - Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter(), Angle::fromDegrees(90)};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::ENEMY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 1.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_partially_blocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().enemyGoalCenter() - Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter() + Vector(0, 0.25), Angle::fromDegrees(45)};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::ENEMY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.5, 0.01);
}

TEST(CalcBestShotTest, calc_open_enemy_net_percentage_with_fully_blocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().enemyGoalCenter() - Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter(), Angle::zero()};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::FRIENDLY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_unblocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().friendlyGoalCenter() + Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter(), Angle::fromDegrees(90)};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::FRIENDLY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 1.0, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_partially_blocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().friendlyGoalCenter() + Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter() + Vector(0, 0.25), Angle::fromDegrees(45)};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::FRIENDLY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.5, 0.01);
}

TEST(CalcBestShotTest, calc_open_friendly_net_percentage_with_fully_blocked_net)
{
    World world       = ::TestUtil::createBlankTestingWorld();
    Field field       = Field::createSSLDivisionBField();
    Point shot_origin = world.field().enemyGoalCenter() + Vector(0.5, 0);
    Shot shot{world.field().enemyGoalCenter(), Angle::zero()};

    auto result = calcShotOpenNetPercentage(field, shot_origin, shot, TeamType::ENEMY);

    // We should not be able to find a shot
    EXPECT_NEAR(result, 0.0, 0.01);
}

TEST(CalcBestShotTest, test_calc_most_open_seg_no_obstacles)
{
    std::vector<Circle> obs = {};
    Segment ref_segment     = Segment(Point(202, 15), Point(202, -15));
    Point origin            = Point(0, 0);

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(origin, ref_segment, obs);

    EXPECT_EQ((ref_segment.getStart() - origin).orientation() -
                  (ref_segment.getEnd() - origin).orientation(),
              open_shot->getOpenAngle());
    EXPECT_EQ(ref_segment.midPoint(), open_shot->getPointToShootAt());
}

TEST(CalcBestShotTest, test_calc_most_open_seg_obstacle_center_obstacle)
{
    Circle obst1 = Circle(Point(100, 0), 0.5);

    std::vector<Circle> obs = {obst1};
    auto open_shot          = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(202, 15), Point(202, -15)), obs);
    EXPECT_NEAR(open_shot->getOpenAngle().toRadians(), 0.069121, 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().x(), Point(202, 8.00501).x(), 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().y(), Point(202, 8.00501).y(), 0.001);
}

TEST(CalcBestShotTest, test_calc_most_open_seg)
{
    Circle obst1 = Circle(Point(200, 1), 0.5);
    Circle obst2 = Circle(Point(200, 1.2), 0.5);
    Circle obst3 = Circle(Point(200, -3), 0.5);
    Circle obst4 = Circle(Point(200, -10), 0.5);
    Circle obst5 = Circle(Point(200, 10), 0.5);

    std::vector<Circle> obs = {obst1, obst2, obst3, obst4, obst5};
    auto open_shot          = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(202, 15), Point(202, -15)), obs);
    EXPECT_NEAR(open_shot->getOpenAngle().toRadians(), 0.038961, 0.0001);
    EXPECT_NEAR(open_shot->getPointToShootAt().x(), Point(202, 5.65572).x(), 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().y(), Point(202, 5.65572).y(), 0.001);
}

TEST(CalcBestShotTest, test_calc_most_open_seg_line_of_obstacles_half_blocked)
{
    Segment ref_seg = Segment(Point(10, -10), Point(10, 10));

    // Create a complete line of obstacles
    std::vector<Circle> obs;

    for (int i = -11; i < 0; i++)
    {
        obs.push_back(Circle(Point(5, i), 0.5));
    }

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(Point(0, 0), ref_seg, obs);
    EXPECT_NEAR(open_shot->getOpenAngle().toRadians(), 0.884578, 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().x(), 10.0, 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().y(), 4.5024, 0.001);
}

TEST(CalcBestShotTest, test_calc_most_open_seg_line_of_obstacles_complete_blocked)
{
    Segment ref_seg = Segment(Point(10, -10), Point(10, 10));

    // Create a complete line of obstacles
    std::vector<Circle> obs;

    for (int i = -12; i < 12; i++)
    {
        obs.push_back(Circle(Point(5, i), 0.5));
    }

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(Point(0, 0), ref_seg, obs);
    EXPECT_FALSE(open_shot.has_value());
}

TEST(CalcBestShotTest, test_calc_most_open_seg_touching_blocking_obstacle)
{
    Segment ref_seg = Segment(Point(10, -5), Point(10, 5));

    // Create a complete line of obstacles
    std::vector<Circle> obs;

    obs.push_back(Circle(Point(0.5, 0), 0.5));

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(Point(0, 0), ref_seg, obs);
    EXPECT_FALSE(open_shot.has_value());
}

TEST(CalcBestShotTest, test_calc_most_open_seg_close_blocking_obstacle)
{
    Segment ref_seg = Segment(Point(10, -5), Point(10, 5));

    // Create a complete line of obstacles
    std::vector<Circle> obs;

    obs.push_back(Circle(Point(0.55, 0), 0.5));

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(Point(0, 0), ref_seg, obs);
    EXPECT_FALSE(open_shot.has_value());
}

TEST(CalcBestShotTest, test_open_shot_with_a_dense_wall_of_obstacles)
{
    std::vector<Circle> obs;
    obs.push_back(Circle(Point(3, -0.09), 0.09));
    obs.push_back(Circle(Point(3, 0), 0.09));
    obs.push_back(Circle(Point(3, 0.09), 0.09));
    // Using an obstacle radius of 0.1 passes, but 0.09 fails. Interesting...
    auto testpair_opt = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(4.5, -0.15), Point(4.5, 0.15)), obs);
    // We do not expect to get a result
    EXPECT_FALSE(testpair_opt.has_value());
}

TEST(CalcBestShotTest, test_calc_open_shot_with_a_dense_wall_of_obstacles_2)
{
    std::vector<Circle> obs;
    obs.push_back(Circle(Point(3, 0.05), 0.1));
    obs.push_back(Circle(Point(3, -0.05), 0.1));

    auto testpair_opt = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(4.5, -0.15), Point(4.5, 0.15)), obs);
    // We do not expect to get a result
    EXPECT_FALSE(testpair_opt.has_value());
}

TEST(CalcBestShotTest, test_calc_most_open_seg_obstacles_behind)
{
    Segment ref_seg = Segment(Point(10, -10), Point(10, 10));
    Point reference = Point(0, 0);
    // Create a complete line of obstacles
    std::vector<Circle> obs;

    for (int i = -12; i < 12; i++)
    {
        obs.push_back(Circle(Point(-5, i), 0.5));
    }

    auto open_shot = calcMostOpenDirectionFromCircleObstacles(reference, ref_seg, obs);
    EXPECT_EQ(open_shot->getOpenAngle(),
              (ref_seg.getStart() - reference)
                  .orientation()
                  .minDiff((ref_seg.getEnd() - reference).orientation())
                  .abs());
}
TEST(CalcBestShotTest,
     test_calc_most_open_seg_obstacles_behind_with_real_obstacles_in_front)
{
    Segment ref_seg = Segment(Point(10, -10), Point(10, 10));
    Point reference = Point(0, 0);
    // Create a complete line of obstacles
    std::vector<Circle> obs;

    for (int i = -12; i < 12; i++)
    {
        obs.push_back(Circle(Point(-5, i), 0.5));
    }
    // Blocking obstacles-
    obs.push_back(Circle(Point(8, 0), 1));
    obs.push_back(Circle(Point(8, 1), 1));
    auto open_shot = calcMostOpenDirectionFromCircleObstacles(reference, ref_seg, obs);
    EXPECT_EQ(open_shot->getOpenAngle(), Angle::fromRadians(0.66007033222938283));
    EXPECT_NEAR(open_shot->getPointToShootAt().x(), Point(10, -5.629940788).x(), 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().y(), Point(10, -5.629940788).y(), 0.001);
}
TEST(CalcBestShotTest, test_calc_most_open_seg_robot_parameter_version)
{
    Segment ref_seg = Segment(Point(10, -10), Point(10, 10));
    Point reference = Point(0, 0);
    // Create a complete line of obstacles
    std::vector<Robot> robots;

    for (int i = -12; i < 12; i++)
    {
        robots.push_back(Robot(i, Point(-5, i), Vector(0, 0), Angle::fromRadians(0),
                               AngularVelocity::fromRadians(0),
                               Timestamp::fromSeconds(0)));
    }
    // Blocking obstacles-
    robots.push_back(Robot(13, Point(8, 0), Vector(0, 0), Angle::fromRadians(0),
                           AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0)));
    robots.push_back(Robot(14, Point(8, 0), Vector(0, 0), Angle::fromRadians(0),
                           AngularVelocity::fromRadians(0), Timestamp::fromSeconds(0)));
    auto open_shot = calcMostOpenDirectionFromRobotObstacles(reference, ref_seg, robots);
    EXPECT_NEAR(open_shot->getOpenAngle().toRadians(), 0.774148, 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().x(), Point(10, -5.05625).x(), 0.001);
    EXPECT_NEAR(open_shot->getPointToShootAt().y(), Point(10, -5.05625).y(), 0.001);
}
TEST(CalcBestShotTest, test_calc_open_shot_circles)
{
    std::vector<Circle> obs;
    obs.clear();
    obs.push_back(Circle(Point(-9, 10), 1.0));
    obs.push_back(Circle(Point(9, 10), 1.0));

    auto testshot = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(10, 10), Point(-10, 10)), obs);

    // We expect to get a result
    EXPECT_TRUE(testshot->getOpenAngle() != Angle::fromDegrees(0));

    EXPECT_TRUE(
        (testshot->getPointToShootAt().toVector().normalize() - Vector(0, 1)).length() <
        0.0001);
    EXPECT_NEAR(75.449, testshot->getOpenAngle().toDegrees(), 1e-4);

    obs.clear();
    obs.push_back(Circle(Point(-4, 6), 1.0));
    obs.push_back(Circle(Point(6, 8), 1.0));
    obs.push_back(Circle(Point(4, 10), 1.0));

    testshot = calcMostOpenDirectionFromCircleObstacles(
        Point(0, 0), Segment(Point(10, 10), Point(-10, 10)), obs);

    // We expect to get a result
    EXPECT_TRUE(testshot->getOpenAngle() != Angle::fromDegrees(0));

    EXPECT_TRUE((testshot->getPointToShootAt().toVector().normalize() -
                 Point(-0.092577, 0.995702).toVector())
                    .length() < 0.0001);
    EXPECT_NEAR(42.1928, testshot->getOpenAngle().toDegrees(), 1e-4);
}
