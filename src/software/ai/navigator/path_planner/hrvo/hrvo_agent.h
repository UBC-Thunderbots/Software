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
#include "software/geom/algorithms/nearest_neighbor_search.hpp"


class HRVOAgent : public Agent {
public:
    /**
     * Constructor
     *
     * @param robot_id	            The robot id for this agent.
     * @prarm robot_state           The robots current state
     * @param type	  	            The team side for this agent (friendly or enemy).
     * @param radius                The radius of this agent.
     * @param path                  The path of this agent
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     */

    HRVOAgent(RobotId robot_id, const RobotState &robot_state, TeamSide side, RobotPath &path,
              double radius, double max_speed, double max_accel, double max_radius_inflation);


    // check double -> Duration conversion
    void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                         const World &world, Duration time_step) override;


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
    void computeVelocityObstacles(std::map<RobotId, std::shared_ptr<Agent>> &robots, Duration time_step);

    /**
     * Create the velocity obstacle which other_agent should see for this Agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;
    /**
     * compute the neighbours of this robot
     * @param robots
     * @return the robot simualator ids
     */
    std::vector<RobotId> computeNeighbors(std::map<RobotId, std::shared_ptr<Agent>> &robots);

    // GETTERS
    std::vector<VelocityObstacle> getVelocityObstacles();

    std::optional<ObstaclePtr> getBallObstacle();


    class Candidate {
    public:
        explicit Candidate(Vector velocity, int index_1, int index_2) :
                velocity(velocity),
                obstacle_indexes(std::pair(
                        index_1,
                        index_2)
                ) {}

        // The velocity of the candidate.
        Vector velocity;
        // pair of (potentially) intersecting velocity obstacles
        // from which we can calculate the candidate velocity
        std::pair<int, int> obstacle_indexes;

        static constexpr double MIN_PREF_SPEED_MULTIPLIER = 0.5;
    };

    /**
     * Returns the first velocity obstacle intersected by the Candidate point in
     * velocity_obstacles_
     *
     * @param candidate the candidate point to check against all velocity obstacles in
     * velocity_obstacles_
     *
     * @return the index of the first velocity obstacle that the given candidate point
     * intersects in velocity_obstacles_, or std::nullopt if the candidate does not
     * intersect any
     */
    std::optional<int> findIntersectingVelocityObstacle(const Candidate &candidate) const;


    /**
     * Returns true if the given candidate point doesn't intersct any obstacle in
     * velocity_obstacles_ and is faster than the minimum preferred speed.
     *
     * @param candidate	the candidate point to consider
     *
     * @return true if it the candidate point doesn't intersect any obstacle and is fast
     */
    bool isIdealCandidate(const Candidate &candidate) const;

    /**
     * Returns true if the given candidate point is faster than the minimum preferred
     * speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is faster or equal to the minimum preferred speed,
     * false otherwise
     */
    bool isCandidateFast(const Candidate &candidate) const;

    /**
     * Returns true if the candidate point is slower than the minimum preferred speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is slower than the minimum preferred speed, false if
     * the speed is faster or the equal to the minimum preferred speed.
     */
    bool isCandidateSlow(const Candidate &candidate) const;

    /**
     * Returns true if the candidate is faster than the current new_velocity_.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate point is faster than new_velocity_, false if the
     * candidate is as fast or slower
     */
    bool isCandidateFasterThanCurrentSpeed(const Candidate &candidate) const;


protected:
    RobotNavigationObstacleFactory obstacle_factory;
    std::vector<VelocityObstacle> velocity_obstacles;
    std::vector<ObstaclePtr> static_obstacles;
    std::optional<ObstaclePtr> ball_obstacle;

    static constexpr double PREF_SPEED_SCALE = 0.85;
    static constexpr double DECEL_DIST_MULTIPLIER = 1.2;
    static constexpr double DECEL_PREF_SPEED_MULTIPLIER = 0.6;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    static constexpr double MAX_NEIGHBOR_SEARCH_DIST = 2.5;
    static constexpr double MAX_NEIGHBORS_DIV_B = 12;
};


