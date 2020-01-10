#pragma once

#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/util.h"
#include "software/new_geom/point.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/world.h"

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
    std::vector<Obstacle> getObstaclesFromMotionConstraints(
        const std::set<MotionConstraint> &motion_constraints, const World &world);

    /**
     * Gets an obstacle representing the given robot
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param robot The robot to get a representative obstacle for
     *
     * @return An obstacle representing the given robot
     */
    Obstacle getVelocityObstacleFromRobot(const Robot &robot);

    /**
     * Gets a list of obstacles representing the given team
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     *
     * @param team The team to get representative obstacles for
     *
     * @return A list of obstacles representing the given team
     */
    std::vector<Obstacle> getVelocityObstaclesFromTeam(const Team &team);

   private:
    std::shared_ptr<const ObstacleFactoryConfig> config;
};
