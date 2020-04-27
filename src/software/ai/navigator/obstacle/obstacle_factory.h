#pragma once

#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/util.h"
#include "software/new_geom/point.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/world/world.h"

/**
 * The obstacle factory creates obstacles with an additional robot radius for navigation
 */
class ObstacleFactory
{
   public:
    ObstacleFactory() = delete;

    /**
     * Create an ObstacleFactory with the given configuration
     *
     * @param config The configuration used to determine how obstacles should be generated
     */
    ObstacleFactory(std::shared_ptr<const ObstacleFactoryConfig> config);

    /**
     * Create obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<Obstacle> createObstaclesFromMotionConstraints(
        const std::set<MotionConstraint> &motion_constraints, const World &world);

    /**
     * Create obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<Obstacle> createObstaclesFromMotionConstraint(
        const MotionConstraint &motion_constraint, const World &world);

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
    Obstacle createVelocityObstacleFromRobot(const Robot &robot);

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
    std::vector<Obstacle> createVelocityObstaclesFromTeam(const Team &team);

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     * @param radius_scaling how much to scale the radius
     *
     * @return obstacle around the robot
     */
    Obstacle createRobotObstacle(const Point &robot_position,
                                 const double radius_scaling = 1.0);

    /**
     * Create circle obstacle around ball
     *
     * @param ball_position ball position to make obstacle around
     *
     * @return obstacle around the ball
     */
    Obstacle createBallObstacle(const Point &ball_position);

    /**
     * Create rectangle-shaped obstacle
     *
     * @param rectangle Rectangle to make obstacle with
     *
     * @return rectangular obstacle
     */
    Obstacle createRectangleObstacle(const Rectangle &rectangle);

   private:
    std::shared_ptr<const ObstacleFactoryConfig> config;
    double shape_expansion_amount;

    /**
     * Create circle obstacle directly from a Circle
     * Note: this helper function does not add a robot radius
     *
     *@param circle Circle
     *
     * @return circle shaped obstacle
     */
    Obstacle createObstacle(const Circle &circle);

    /**
     * Create polygon obstacle directly from a Polygon
     * Note: this helper function does not add a robot radius
     *
     *@param polygon Polygon
     *
     * @return polygon shaped obstacle
     */
    Obstacle createObstacle(const Polygon &polygon);
};
