#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/time/duration.h"
#include "software/geom/vector.h"
#include "software/geom/algorithms/intersection.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"
#include "software/world/world.h"
#include "proto/primitive.pb.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

class HRVOAgent : Agent {
public:
    /**
     * Constructor
     *
     * @param robot_id	            The robot id for this agent.
     * @prarm robot_state            The robots current state
     * @param type	  	            The team side for this agent (friendly or enemy).
     * @param radius                The radius of this agent.
     * @param path                  The path of this agent
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     */

    HRVOAgent(RobotId robot_id, const RobotState &robot_state, TeamSide side, RobotPath &path,
              double radius, double min_radius, double max_speed, double max_accel, double max_radius_inflation);


    // TODO
    // check double -> Duration conversion
    void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                    const World &world, Duration time_step);


    /**
     * Computes the new velocity of this agent.
     * @param agents a map of offset robot ids to agents
     */
    void computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots, Duration time_step) override;

    /**
     * Computes the preferred velocity of this agent.
     * relies on
     * pref_speed, velocity and position
     *
     * updates pref_velocity
     */
    Vector computePreferredVelocity(Duration time_step) override;

    /**
     * Compute all the velocity obstacles that this Agent should take into account and
     * add it to `velocityObstacles_`.
     * this only considers making VO's for relevant agents in the `neighbours_` field.
     *
     * relies on
     * `ball_obstacle`, `neighbours_`
     */
    void computeVelocityObstacles(std::map<unsigned int, std::shared_ptr<Agent>> &robots, Duration time_step);

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;

    /**
     * Create the hybrid reciprocal velocity obstacle which other_agent should see for
     * this Agent
     *
     * @param other_agent The Agent which this hybrid reciprocal velocity obstacle is
     * being generated for
     * @return The hybrid reciprocal velocity obstacle which other_agent should see for
     * this Agent
     */
    VelocityObstacle createHybridReciprocalVelocityObstacle(const HRVOAgent &other_agent);

    /**
     * compute the neighbours of this robot
     * @param robots
     * @return the robot simualator ids
     */
    std::vector<unsigned int> computeNeighbors(std::map<unsigned int, std::shared_ptr<Agent>> &robots);

    class Candidate
    {
    public:

        // TODO
        // fix to use constructor instead of setters
        // move to separate file
        explicit Candidate(Vector velocity) : velocity(velocity), obstacles(std::pair(VelocityObstacle(Vector(0, 0), Vector(0, 0), Vector(0, 0)),
                                          VelocityObstacle(Vector(0, 0), Vector(0, 0), Vector(0, 0)))) {}

        // The velocity of the candidate.
        Vector velocity;
        // pair of (potentially) intersecting velocity obstacles
        // from which we can calculate the candidate velocity
        std::pair<VelocityObstacle, VelocityObstacle> obstacles;
    };


protected:
    Vector new_velocity;

    RobotNavigationObstacleFactory obstacle_factory;

    std::vector<VelocityObstacle> velocity_obstacles;
    std::vector<ObstaclePtr> static_obstacles;
    std::optional<ObstaclePtr> ball_obstacle;

    static constexpr double MIN_PREF_SPEED_MULTIPLIER = 0.5;
    static constexpr double DECEL_DIST_MULTIPLIER = 1.2;
    static constexpr double DECEL_PREF_SPEED_MULTIPLIER = 0.6;
    static constexpr double PREF_SPEED_SCALE = 0.85;
};


