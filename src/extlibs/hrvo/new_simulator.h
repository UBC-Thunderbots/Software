#pragma once

#include <fstream>
#include <limits>
#include <vector>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"


class new_simulator {
    // List of agents (robots) in this simulation
    std::vector<std::shared_ptr<Agent>> agents;

    // robot id to agent index
    std::map<unsigned int, unsigned int> friendly_robot_id_map;
    std::map<unsigned int, unsigned int> enemy_robot_id_map;

    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr float FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05f;
    static constexpr float ENEMY_ROBOT_RADIUS_MAX_INFLATION    = 0.06f;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    static constexpr float GOAL_RADIUS_SCALE = 1.05f;

    // How much larger should the goal radius be (in meters). This is added as a safety
    // tolerance so robots do not accidentally enter the minimum distance threshold.
    // NOTE: This value must be >= 0
    static constexpr float BALL_AGENT_RADIUS_OFFSET = 0.1f;

    // The scale multiple of max robot speed which the preferred speed will be set at.
    // pref_speed = max_speed * PREF_SPEED_SCALE
    // NOTE: This scale multiple must be <= 1
    static constexpr float PREF_SPEED_SCALE = 0.85f;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    static constexpr float MAX_NEIGHBOR_SEARCH_DIST = 2.5f;
    // The maximum number of neighbors/agents to consider when drawing velocity obstacles.
    static constexpr unsigned int MAX_NEIGHBORS = 15;
};