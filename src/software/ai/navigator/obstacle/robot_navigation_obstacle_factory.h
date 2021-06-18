#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/point.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

/**
 * The RobotNavigationObstacleFactory creates obstacles for navigation with a robot
 * NOTE: All obstacles created include at least an additional robot radius margin on all
 * sides of the obstacle
 */
class RobotNavigationObstacleFactory
{
   public:
    RobotNavigationObstacleFactory() = delete;

    /**
     * Create an RobotNavigationObstacleFactory with the given configuration
     *
     * @param config The configuration used to determine how obstacles should be generated
     */
    RobotNavigationObstacleFactory(
        std::shared_ptr<const RobotNavigationObstacleConfig> config);

    /**
     * Create obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createFromMotionConstraints(
        const std::set<MotionConstraint> &motion_constraints, const World &world) const;

    /**
     * Create obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createFromMotionConstraint(
        const MotionConstraint &motion_constraint, const World &world) const;

    /**
     * Create an obstacle representing the given robot
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param robot The robot to get a representative obstacle for
     *
     * @return An obstacle representing the given robot
     */
    ObstaclePtr createFromRobot(const Robot &robot) const;

    /**
     * Create a list of obstacles representing the given team
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param team The team to get representative obstacles for
     *
     * @return A list of obstacles representing the given team
     */
    std::vector<ObstaclePtr> createFromTeam(const Team &team) const;

    /**
     * Create a list of obstacles that stop enemy robot collision. These obstacles are
     * scaled down if friendly_robot_speed is below a threshold set in the config to allow
     * slow collisions with enemy robots
     *
     * @param enemy_team The enemy team to get representative obstacles for
     * @param friendly_robot_speed The speed of the current friendly robot
     *
     * @return A list of obstacles representing the given team
     */
    std::vector<ObstaclePtr> createEnemyCollisionAvoidance(
        const Team &enemy_team, double friendly_robot_speed) const;

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createFromRobotPosition(const Point &robot_position) const;

    /**
     * Create circle obstacle around ball
     *
     * @param ball_position ball position to make obstacle around
     *
     * @return obstacle around the ball
     */
    ObstaclePtr createFromBallPosition(const Point &ball_position) const;

    /**
     * Returns an obstacle for the shape
     * NOTE: as with all other obstacles created by RobotNavigationObstacleFactory, the
     * shapes are expanded on all sides to account for the radius of the robot
     *
     * @param The shape to expand
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromShape(const Circle &circle) const;
    ObstaclePtr createFromShape(const Polygon &polygon) const;

   private:
    std::shared_ptr<const RobotNavigationObstacleConfig> config;
    double robot_radius_expansion_amount;

    /**
     * Returns an obstacle for the field_rectangle expanded on all sides to account for
     * the size of the robot. If a side of the field_rectangle lies along a field line,
     * then it is expanded out to the field boundary
     *
     * @param field_rectangle The rectangle to make obstacle
     * @param field_lines The rectangle representing field lines
     * @param field_boundary The rectangle representing field boundary
     * @param additional_expansion_amount (optional) The amount to expand all sides of the
     * rectangle in addition to the robot radius
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromFieldRectangle(const Rectangle &field_rectangle,
                                         const Rectangle &field_lines,
                                         const Rectangle &field_boundary,
                                         double additional_expansion_amount = 0.0) const;
};
