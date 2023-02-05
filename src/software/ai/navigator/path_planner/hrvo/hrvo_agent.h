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



    void updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                    const World &world, double time_step);

    /*
     * updates the radius based on current velocity
     */
    void updateRadiusFromVelocity() override;

    /*
     * get robots path
     */
    const RobotPath &getPath() override;

    /**
     * Computes the new velocity of this agent.
     * @param friendlies hrvo agents for representing the friendly team
     * @param enemies linear velocity agents for representing the enemy team
     */
    void computeNewVelocity(std::map<int, HRVOAgent> &friendlies, std::map<int, LVAgent> &enemies, Duration time_step);

    /**
     * Computes the preferred velocity of this agent.
     * relies on
     * pref_speed, velocity and position
     *
     * updates pref_velocity
     */
    Vector computePreferredVelocity(double time_step);

    /**
     * Compute all the velocity obstacles that this Agent should take into account and
     * add it to `velocityObstacles_`.
     * this only considers making VO's for relevant agents in the `neighbours_` field.
     *
     * relies on
     * `ball_obstacle`, `neighbours_`
     */
    std::vector<VelocityObstacle> computeVelocityObstacles(std::map<int, Agent> &robots, double time_step) override;

    /**
     * Computes the most direct linear velocity for this agent.
     * @param agents is unused
     */
    void computeLinearVelocity(Duration time_step);

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

    class Candidate
    {
    public:

        // FOR NOW like this kekw
        Candidate() : obstacles(std::pair(VelocityObstacle(Vector(0, 0), Vector(0, 0), Vector(0, 0)),
                                          VelocityObstacle(Vector(0, 0), Vector(0, 0), Vector(0, 0)))) {}

        // The velocity of the candidate.
        Vector velocity;
        // pair of (potentially) intersecting velocity obstacles
        // from which we can calculate the candidate velocity
        std::pair<VelocityObstacle, VelocityObstacle> obstacles;
    };


protected:
    // robot id of this Agent
    RobotId robot_id;
    RobotState robot_state;
    // whether this Agent is FRIENDLY or ENEMY
    // keeping this for now, even tho we have assumption that friendlies are hrvo.
    // maybe at some point we can predict the strategy used by enemies
    TeamSide side;
    // This agent's current actual radius
    double radius;
    // The path of this Agent
    RobotPath path;

    std::vector<VelocityObstacle> velocity_obstacles;
};


