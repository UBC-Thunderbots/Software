#include "software/ai/navigator/trajectory/trajectory_planner.h"

#include <gtest/gtest.h>

#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/geom/algorithms/contains.h"
#include "software/test_util/test_util.h"

class TrajectoryPlannerTest : public testing::Test
{
   protected:
    TrajectoryPlannerTest()
        : world(TestUtil::createBlankTestingWorld(TbotsProto::FieldType::DIV_B)),
          obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()),
          constraints(3.0, 3.0, 3.0),
          stop_cmd_constraints(1.5, 3.0, 3.0)
    {
        // Set the inflation factor as a constant to avoid future changes of the constant
        // breaking the tests.
        TbotsProto::RobotNavigationObstacleConfig config;
        config.set_robot_obstacle_inflation_factor(1.0);
        obstacle_factory = RobotNavigationObstacleFactory(config);

        // Robot at origin
        Point robot_obstacle_position(0, 0);
        robot_obstacle = obstacle_factory.createStaticObstacleFromRobotPosition(
            robot_obstacle_position);

        friendly_defense_area_obstacle =
            obstacle_factory.createObstaclesFromMotionConstraints(
                {TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA}, *world)[0];

        enemy_half_obstacle = obstacle_factory.createObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::ENEMY_HALF}, *world)[0];

        center_circle_obstacle = obstacle_factory.createObstaclesFromMotionConstraints(
            {TbotsProto::MotionConstraint::CENTER_CIRCLE}, *world)[0];

        enemy_half_without_center_circle_obstacle =
            obstacle_factory.createObstaclesFromMotionConstraints(
                {TbotsProto::MotionConstraint::ENEMY_HALF_WITHOUT_CENTRE_CIRCLE},
                *world)[0];
    }

    void verifyNoCollision(const TrajectoryPath& trajectory,
                           const std::vector<ObstaclePtr>& obstacles)
    {
        for (int i = 0; i <= NUM_SUB_POINTS; ++i)
        {
            double t_sec = i *
                           std::min(trajectory.getTotalTime(), MAX_COLLISION_CHECK_TIME) /
                           NUM_SUB_POINTS;
            Point pos = trajectory.getPosition(t_sec);
            for (const ObstaclePtr& obstacle : obstacles)
            {
                ASSERT_FALSE(obstacle->contains(pos))
                    << "Trajectory collides with obstacle at t=" << t_sec
                    << ". pos=" << pos << " obstacle=" << obstacle;
            }
        }
    }

    void verifyTrajectoryIsWithinRectangle(const TrajectoryPath& trajectory,
                                           const Rectangle& rectangle)
    {
        for (int i = 0; i <= NUM_SUB_POINTS; ++i)
        {
            double t_sec = i * trajectory.getTotalTime() / NUM_SUB_POINTS;
            Point pos    = trajectory.getPosition(t_sec);
            ASSERT_TRUE(contains(rectangle, pos))
                << "Trajectory is not within rectangle at t=" << t_sec << ". pos=" << pos
                << " rectangle=" << rectangle;
        }
    }

    TrajectoryPlanner traj_planner;
    std::shared_ptr<World> world;
    RobotNavigationObstacleFactory obstacle_factory;
    ObstaclePtr robot_obstacle;
    ObstaclePtr friendly_defense_area_obstacle;
    ObstaclePtr center_circle_obstacle;
    ObstaclePtr enemy_half_obstacle;
    ObstaclePtr enemy_half_without_center_circle_obstacle;
    KinematicConstraints constraints;
    KinematicConstraints stop_cmd_constraints;

    static constexpr int NUM_SUB_POINTS              = 30;
    static constexpr double MAX_COLLISION_CHECK_TIME = 2.0;
};

TEST_F(TrajectoryPlannerTest, test_traj_avoid_robot_obstacle)
{
    Point start_pos(-1.0, 0.0);
    Point destination(1.0, 0.0);
    Rectangle valid_traj_rectangle({-1.01, -3 * ROBOT_MAX_RADIUS_METERS},
                                   {1.01, 3 * ROBOT_MAX_RADIUS_METERS});
    std::vector obstacles = {robot_obstacle};

    auto traj_path =
        traj_planner.findTrajectory(start_pos, destination, Vector(), constraints,
                                    obstacles, world->field().fieldBoundary());

    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);
    EXPECT_EQ(traj_path->getDestination(), destination);
    verifyNoCollision(traj_path.value(), obstacles);
    verifyTrajectoryIsWithinRectangle(traj_path.value(), valid_traj_rectangle);
}

TEST_F(TrajectoryPlannerTest, test_traj_avoid_friendly_defense_area)
{
    Point start_pos(-4.0, -1.5);
    Point destination(-4.0, 1.5);
    Rectangle valid_traj_rectangle({-4.01, -2.0}, {-2.0, 2.0});
    std::vector obstacles = {friendly_defense_area_obstacle};

    auto traj_path =
        traj_planner.findTrajectory(start_pos, destination, Vector(), constraints,
                                    obstacles, world->field().fieldBoundary());

    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);
    EXPECT_EQ(traj_path->getDestination(), destination);
    verifyNoCollision(traj_path.value(), obstacles);
    verifyTrajectoryIsWithinRectangle(traj_path.value(), valid_traj_rectangle);
}

TEST_F(TrajectoryPlannerTest, test_traj_avoid_center_circle)
{
    Point start_pos(-1.0, 0.0);
    Point destination(1.0, 0.0);
    Rectangle valid_traj_rectangle({-1.01, -1.0 - 3 * ROBOT_MAX_RADIUS_METERS},
                                   {1.01, 1.0 + 3 * ROBOT_MAX_RADIUS_METERS});
    std::vector obstacles = {center_circle_obstacle};

    auto traj_path =
        traj_planner.findTrajectory(start_pos, destination, Vector(), constraints,
                                    obstacles, world->field().fieldBoundary());

    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);
    EXPECT_EQ(traj_path->getDestination(), destination);
    verifyNoCollision(traj_path.value(), obstacles);
    verifyTrajectoryIsWithinRectangle(traj_path.value(), valid_traj_rectangle);
}

TEST_F(TrajectoryPlannerTest, test_traj_avoid_center_circle_and_enemy_half)
{
    // Drive down near the center line, on friendly half, while avoiding
    // the center circle and enemy half obstacles.
    // Simmulating robots getting in position during enemy kickoff.
    Point start_pos(-0.3, 1.0);
    Point destination(-0.3, -1.0);
    Rectangle valid_traj_rectangle({-0.8, -1.0 - 3 * ROBOT_MAX_RADIUS_METERS},
                                   {0.0, 1.0 + 3 * ROBOT_MAX_RADIUS_METERS});
    std::vector obstacles = {center_circle_obstacle, enemy_half_obstacle};

    auto traj_path = traj_planner.findTrajectory(start_pos, destination, Vector(),
                                                 stop_cmd_constraints, obstacles,
                                                 world->field().fieldBoundary());

    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);
    EXPECT_EQ(traj_path->getDestination(), destination);
    verifyNoCollision(traj_path.value(), obstacles);
    verifyTrajectoryIsWithinRectangle(traj_path.value(), valid_traj_rectangle);
}

TEST_F(TrajectoryPlannerTest, test_traj_avoid_enemy_half_and_ball_during_friendly_kickoff)
{
    Point start_pos(-0.3, 1.5);
    // Behind the ball in position for robot to pass back
    Point destination(0.3, 0);
    Rectangle valid_traj_rectangle({0.5, -0.5}, {-0.5, 1.6});

    std::vector obstacles = {enemy_half_without_center_circle_obstacle,
                             obstacle_factory.createFromBallPosition(Point(0, 0))};

    auto traj_path = traj_planner.findTrajectory(start_pos, destination, Vector(),
                                                 stop_cmd_constraints, obstacles,
                                                 world->field().fieldBoundary());

    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);
    EXPECT_EQ(traj_path->getDestination(), destination);
    verifyNoCollision(traj_path.value(), obstacles);
    verifyTrajectoryIsWithinRectangle(traj_path.value(), valid_traj_rectangle);
}

TEST_F(TrajectoryPlannerTest, test_traj_escapes_when_starting_inside_obstacle)
{
    // Regression test: a robot that starts *inside* an obstacle must be given a
    // trajectory that drives it out, never a stop primitive (findTrajectory returning
    // nullopt). Previously, if the robot exited the obstacle it started in but was then
    // driven into another obstacle within the unavoidable-collision window (<0.2s) while
    // moving faster than 0.5 m/s, the planner returned nullopt. move_primitive turns that
    // into a stop, leaving the robot frozen inside the motion-constrained area forever,
    // unable to escape.

    // The robot starts just inside start_obstacle's +x edge, moving in +x at 2 m/s. Its
    // momentum makes it exit start_obstacle at ~t=0.05s and reach blocking_obstacle at
    // ~t=0.15s. The blocker is placed exactly where the robot's (tiny) reachable set
    // lands at t=0.15s, so no amount of steering avoids that collision within the 0.2s
    // unavoidable-collision window at >0.5 m/s -- exactly the case that used to make the
    // planner return nullopt (a stop) and leave the robot stuck.
    Point start_pos(0.10, 0.0);
    Point destination(2.0, 0.0);
    Vector initial_velocity(2.0, 0.0);
    KinematicConstraints slow_constraints(2.0, 3.0, 3.0);

    ObstaclePtr start_obstacle =
        obstacle_factory.createStaticObstacleFromRobotPosition(Point(0.0, 0.0));
    ObstaclePtr blocking_obstacle =
        obstacle_factory.createStaticObstacleFromRobotPosition(Point(0.45, 0.0));
    std::vector obstacles = {start_obstacle, blocking_obstacle};

    auto traj_path = traj_planner.findTrajectory(start_pos, destination, initial_velocity,
                                                 slow_constraints, obstacles,
                                                 world->field().fieldBoundary());

    // Must return an escape trajectory rather than a stop primitive.
    ASSERT_TRUE(traj_path.has_value());
    EXPECT_EQ(traj_path->getPosition(0.0), start_pos);

    // The trajectory must actually leave the obstacle the robot started inside.
    bool escapes_starting_obstacle = false;
    for (int i = 0; i <= NUM_SUB_POINTS; ++i)
    {
        double t_sec = i * std::min(traj_path->getTotalTime(), MAX_COLLISION_CHECK_TIME) /
                       NUM_SUB_POINTS;
        if (!start_obstacle->contains(traj_path->getPosition(t_sec)))
        {
            escapes_starting_obstacle = true;
            break;
        }
    }
    EXPECT_TRUE(escapes_starting_obstacle);
}
