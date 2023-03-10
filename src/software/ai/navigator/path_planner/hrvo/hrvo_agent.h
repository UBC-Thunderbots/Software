#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_planner/hrvo/lv_agent.h"
#include "software/ai/navigator/path_planner/hrvo/robot_path.h"
#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/nearest_neighbor_search.hpp"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"
#include "software/world/world.h"


class HRVOAgent : public Agent
{
   public:
    /**
     * Constructor
     *
     * @param robot_id	            The robot id for this agent.
     * @prarm robot_state           The robots current state
     * @param path                  The path of this agent
     * @param radius                The radius of this agent.
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     */
    HRVOAgent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path,
              double radius, double max_speed, double max_accel,
              double max_radius_inflation);


    /**
     * Update the primitive which this agent is currently pursuing.
     *
     * @param new_primitive The new primitive to pursue
     * @param world The world in which the new primitive is being pursued
     * @param time_step the time_step to use to step at
     */
    void updatePrimitive(const TbotsProto::Primitive &new_primitive, const World &world,
                         Duration time_step) override;


    /**
     * Computes the new velocity of this agent.
     *
     * @param robots a map from robot ids to robots, for all robots in the simulation
     */
    void computeNewVelocity(std::map<unsigned int, std::shared_ptr<Agent>> &robots,
                            Duration time_step) override;

    /**
     * Computes the preferred velocity of this agent.
     *
     * @param time_step
     * @return the computed preferred velocity
     */
    Vector computePreferredVelocity(Duration time_step) override;

    /**
     * Compute all the velocity obstacles that this Agent should take into account and
     * add it to `velocityObstacles`.
     *
     * @param the robots nearest neighbours
     */
    void computeVelocityObstacles(std::map<RobotId, std::shared_ptr<Agent>> &robots);

    /**
     * Create the VO for the given agent, relative to this agent
     *
     * @param other_agent The Agent which this velocity obstacle is being generated for
     * @return The velocity obstacle which other_agent should see for this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;
    /**
     * compute the neighbours of this robot
     * @param robots
     * @return the robot simulator ids
     */
    std::vector<RobotId> computeNeighbors(
        std::map<RobotId, std::shared_ptr<Agent>> &robots);

    /**
     * get the velocity obstacles that this agent sees
     * @return a list of velocity obstacles
     */
    std::vector<VelocityObstacle> getVelocityObstacles();

    /**
     * get the velocity obstacle for the ball that the robot sees
     * @return an o obstacle pointer
     */
    std::optional<ObstaclePtr> getBallObstacle();


    class CandidateVelocity
    {
       public:
        /**
         * Constructor. all candidate velocities occur at the intersections of vo's.
         * @param velocity the velocity for this candidate
         * @param index_1 the index of the first velocity obstacle
         * @param index_2 the index of the second velocity obstacle
         */
        explicit CandidateVelocity(Vector velocity, int index_1, int index_2)
            : velocity(velocity), obstacle_indexes(std::pair(index_1, index_2))
        {
        }

        // The velocity of the candidate.
        Vector velocity;
        // pair of (potentially) intersecting velocity obstacles
        // from which we can calculate the candidate velocity
        std::pair<int, int> obstacle_indexes;
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
    std::optional<int> findIntersectingVelocityObstacle(
        const CandidateVelocity &candidate) const;


    /**
     * Returns true if the given candidate point doesn't intersct any obstacle in
     * velocity_obstacles_ and is faster than the minimum preferred speed.
     *
     * @param candidate	the candidate point to consider
     *
     * @return true if it the candidate point doesn't intersect any obstacle and is fast
     */
    bool isIdealCandidate(const CandidateVelocity &candidate) const;

    /**
     * Returns true if the given candidate point is faster than the minimum preferred
     * speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is faster or equal to the minimum preferred speed,
     * false otherwise
     */
    bool isCandidateFast(const CandidateVelocity &candidate) const;

    /**
     * Returns true if the candidate point is slower than the minimum preferred speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is slower than the minimum preferred speed, false if
     * the speed is faster or the equal to the minimum preferred speed.
     */
    bool isCandidateSlow(const CandidateVelocity &candidate) const;

    /**
     * Returns true if the candidate is faster than the current new_velocity_.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate point is faster than new_velocity_, false if the
     * candidate is as fast or slower
     */
    bool isCandidateFasterThanCurrentSpeed(const CandidateVelocity &candidate) const;


   protected:
    RobotNavigationObstacleFactory obstacle_factory;
    std::vector<VelocityObstacle> velocity_obstacles;
    std::vector<ObstaclePtr> static_obstacles;
    std::optional<ObstaclePtr> ball_obstacle;

    static constexpr double PREF_SPEED_SCALE            = 0.85;
    static constexpr double DECEL_DIST_MULTIPLIER       = 1.2;
    static constexpr double DECEL_PREF_SPEED_MULTIPLIER = 0.6;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    static constexpr double MAX_NEIGHBOR_SEARCH_DIST = 2.5;

    // The maximum number of neighbours to look for.
    static constexpr double MAX_NEIGHBORS_DIV_B = 12;

    static constexpr double MIN_PREF_SPEED_MULTIPLIER = 0.5;
};
